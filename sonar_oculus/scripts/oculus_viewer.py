#!/usr/bin/env python3
import numpy as np
import cv2
from scipy.interpolate import interp1d

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sonar_oculus.msg import OculusPing
from sensor_msgs.msg import Image

REVERSE_Z = 1


class OculusViewer(Node):
    def __init__(self):
        super().__init__('oculus_viewer')
        
        # Declare parameters
        self.declare_parameter('raw', False)
        self.declare_parameter('colormap', 2)
        
        # Initialize variables
        self.res = None
        self.height = None
        self.rows = None
        self.width = None
        self.cols = None
        self.map_x = None
        self.map_y = None
        self.f_bearings = None
        
        self.bridge = CvBridge()
        
        # Create publisher and subscriber
        self.img_pub = self.create_publisher(Image, '/sonar_oculus_node/image', 10)
        self.ping_sub = self.create_subscription(
            OculusPing,
            '/sonar_oculus_node/ping',
            self.ping_callback,
            10
        )
        
        self.get_logger().info('Oculus viewer started')
    
    def to_rad(self, bearing):
        return bearing * np.pi / 18000
    
    def generate_map_xy(self, ping):
        _res = ping.range_resolution
        _height = ping.num_ranges * _res
        _rows = ping.num_ranges
        _width = np.sin(
            self.to_rad(ping.bearings[-1] - ping.bearings[0]) / 2) * _height * 2
        _cols = int(np.ceil(_width / _res))

        if (self.res == _res and self.height == _height and 
            self.rows == _rows and self.width == _width and self.cols == _cols):
            return
        
        self.res = _res
        self.height = _height
        self.rows = _rows
        self.width = _width
        self.cols = _cols

        bearings = self.to_rad(np.asarray(ping.bearings, dtype=np.float32))
        self.f_bearings = interp1d(
            bearings,
            range(len(bearings)),
            kind='linear',
            bounds_error=False,
            fill_value=-1,
            assume_sorted=True)

        XX, YY = np.meshgrid(range(self.cols), range(self.rows))
        x = self.res * (self.rows - YY)
        y = self.res * (-self.cols / 2.0 + XX + 0.5)
        b = np.arctan2(y, x) * REVERSE_Z
        r = np.sqrt(np.square(x) + np.square(y))
        self.map_y = np.asarray(r / self.res, dtype=np.float32)
        self.map_x = np.asarray(self.f_bearings(b), dtype=np.float32)

    def ping_callback(self, msg):
        raw = self.get_parameter('raw').get_parameter_value().bool_value
        cm = self.get_parameter('colormap').get_parameter_value().integer_value
        
        if raw:
            img = self.bridge.imgmsg_to_cv2(msg.ping, desired_encoding='passthrough')
            img = cv2.normalize(
                img, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
            img = cv2.applyColorMap(img, cm)
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(img_msg)
        else:
            self.generate_map_xy(msg)

            img = self.bridge.imgmsg_to_cv2(msg.ping, desired_encoding='passthrough')
            img = np.array(img, dtype=img.dtype, order='F')

            if self.cols > img.shape[1]:
                img.resize(self.rows, self.cols)
            img = cv2.remap(img, self.map_x, self.map_y, cv2.INTER_LINEAR)

            img = cv2.applyColorMap(img, cm)
            img_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            self.img_pub.publish(img_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OculusViewer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
