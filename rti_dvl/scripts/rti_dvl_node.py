#!/usr/bin/env python3
import sys
import os

# Add the script directory to path for pynmea2
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rti_dvl.msg import DVL, Command, BottomTrack

import pynmea2
import serial

SALINITY = {'fresh': 0, 'salt': 35}


class SyncTime(object):
    """Time translator
    - See https://github.com/ethz-asl/cuckoo_time_translator for more details.
    - Implement https://github.com/ros-drivers/urg_node/blob/d2722c60f1b4713bbe1d39f32849090dece0104d/src/urg_c_wrapper.cpp#L1052
    """

    def __init__(self):
        self.hardware_clock = 0.0
        self.last_hardware_time_stamp = 0.0
        self.hardware_clock_adj = 0.0
        self.adj_count = 0
        self.adj_alpha = 0.01

    def sync(self, time_stamp, system_time_stamp):
        delta = time_stamp - self.last_hardware_time_stamp
        self.hardware_clock += delta
        cur_adj = system_time_stamp - self.hardware_clock
        if self.adj_count > 0:
            self.hardware_clock_adj = (
                self.adj_alpha * cur_adj + (1.0 - self.adj_alpha) * self.hardware_clock_adj)
        else:
            self.hardware_clock_adj = cur_adj

        self.adj_count += 1
        self.last_hardware_time_stamp = time_stamp

        stamp = system_time_stamp
        if self.adj_count > 100:
            stamp = self.hardware_clock + self.hardware_clock_adj

            if abs(stamp - system_time_stamp) > 0.1:
                self.adj_count = 0
                self.hardware_clock = 0.0
                self.last_hardware_time_stamp = 0.0
                stamp = system_time_stamp
        return stamp


class RtiDvlNode(Node):
    def __init__(self):
        super().__init__('rti_dvl_node')

        # Declare parameters
        self.declare_parameter('dev', '/tmp/rti_dvl')
        self.declare_parameter('commands', '')
        self.declare_parameter('water', 'fresh')

        self.dev = self.get_parameter('dev').get_parameter_value().string_value
        config_str = self.get_parameter('commands').get_parameter_value().string_value
        self.config = config_str.split(';') if config_str else []
        water = self.get_parameter('water').get_parameter_value().string_value
        self.salinity = SALINITY.get(water, 0)

        # Publishers
        self.dvl_pub = self.create_publisher(DVL, '/rti/body_velocity/raw', 100)
        self.bt_pub = self.create_publisher(BottomTrack, '/rti/bottom_tracking/raw', 100)

        self.dvl = None
        self.cmd = Command()
        self.sync = SyncTime()
        self.reader = pynmea2.NMEAStreamReader(errors='ignore')
        self.bt = None

        # Connect to DVL
        self.connect_to_dvl()

    def write_to_dvl(self, s, duration=0.5):
        self.dvl.write((s.strip() + '\r').encode())
        import time
        time.sleep(duration)

    def connect_to_dvl(self):
        while rclpy.ok():
            try:
                self.get_logger().info(f'Open device {self.dev}')
                self.dvl = serial.Serial(self.dev, dsrdtr=True, rtscts=True, timeout=1.0)
                import time
                time.sleep(1.0)
                break
            except serial.SerialException:
                self.get_logger().error(f'Fail to open device {self.dev}')
                import time
                time.sleep(1.0)

        if not rclpy.ok():
            return

        # STOP
        while self.dvl.in_waiting:
            line = self.dvl.readline()
            if line.strip().startswith(b'STOP'):
                break
            self.write_to_dvl('STOP')
            self.get_logger().info('Trying to stop DVL pinging...')

        # Zero pressure sensor
        self.write_to_dvl('CPZ')
        # Change water salinity
        self.write_to_dvl(f'CWS {self.salinity}')
        # Write extra commands in config file
        for c in self.config:
            if c:
                self.write_to_dvl(c)
                self.get_logger().info('Write to DVL: ' + c)

        # Read DVL commands
        while True:
            self.write_to_dvl('CSHOW')
            self.get_logger().info('Trying to read DVL commands...')

            received = False
            while self.dvl.in_waiting:
                line = self.dvl.readline()
                cp = line.decode('utf-8', errors='ignore').strip().split(' ', 1)
                if len(cp) == 2:
                    c, p = cp
                    if c == 'CEPO':
                        received = True
                    c = c.split('[', 1)[0]
                    if hasattr(self.cmd, c.lower()):
                        setattr(self.cmd, c.lower(), p)
            
            if received:
                break
            else:
                import time
                time.sleep(1.0)

        # Start
        while not self.dvl.in_waiting:
            self.write_to_dvl('START')
            self.get_logger().info('Trying to start DVL pinging...')

    def time_to_stamp(self, time_sec):
        """Convert float seconds to ROS2 Time stamp"""
        sec = int(time_sec)
        nanosec = int((time_sec - sec) * 1e9)
        return Time(seconds=sec, nanoseconds=nanosec).to_msg()

    def run(self):
        while rclpy.ok():
            try:
                char = self.dvl.read()
            except serial.SerialException:
                break
            
            for msg in self.reader.next(char):
                now = self.get_clock().now()
                system_time_stamp = now.nanoseconds / 1e9

                if isinstance(msg, pynmea2.types.rti.RTI01) or isinstance(msg, pynmea2.types.rti.RTI03):
                    stamp = self.sync.sync(msg.time / 100.0, system_time_stamp)

                    # For back compatibility
                    d = DVL()
                    d.header.stamp = self.time_to_stamp(stamp)
                    d.velocity.x = msg.x / 1000.0
                    d.velocity.y = msg.y / 1000.0
                    d.velocity.z = msg.z / 1000.0
                    d.temperature = msg.temperature / 100.0
                    d.altitude = msg.depth / 1000.0
                    d.time = msg.time / 100.0
                    self.dvl_pub.publish(d)

                    self.bt = BottomTrack()
                    self.bt.header.stamp = self.time_to_stamp(stamp)
                    self.bt.command = self.cmd
                    self.bt.velocity.x = msg.x / 1000.0
                    self.bt.velocity.y = msg.y / 1000.0
                    self.bt.velocity.z = msg.z / 1000.0
                    self.bt.temperature = msg.temperature / 100.0
                    self.bt.altitude = msg.depth / 1000.0
                    self.bt.time = msg.time / 100.0
                    self.bt.sample = msg.number

                if isinstance(msg, pynmea2.types.rti.RTI30) or isinstance(msg, pynmea2.types.rti.RTI32):
                    if self.bt is None:
                        continue
                    self.bt.orientation.x = msg.roll
                    self.bt.orientation.y = msg.pitch
                    self.bt.orientation.z = msg.heading
                    self.bt_pub.publish(self.bt)

        if self.dvl:
            self.dvl.close()


def main(args=None):
    rclpy.init(args=args)
    node = RtiDvlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
