// VSieben@slb.com (original author, 2017)
// pvt@mit.edu     (extensions, 2018)
// jwang92@stevens.edu
// Converted to ROS2

#include <algorithm>
#include <arpa/inet.h>
#include <iostream>
#include <math.h>
#include <netdb.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <functional>
#include <memory>
#include <chrono>

#include "Oculus.h"
#include "OculusClient.h"

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sonar_oculus/msg/oculus_fire.hpp>
#include <sonar_oculus/msg/oculus_ping.hpp>

#define BUFLEN 200
#define DATALEN 200000
#define PORT_UDP 52102
#define PORT_TCP 52100

// Create sonar oculus control class
OsClientCtrl sonar;

class SonarOculusNode : public rclcpp::Node
{
public:
  SonarOculusNode() : Node("sonar_oculus_node"), latest_id_(0), part_number_(0)
  {
    // Declare parameters with defaults
    this->declare_parameter<int>("mode", 1);           // 1 => ~750khz, 2 => ~1.2Mhz
    this->declare_parameter<int>("ping_rate", 0);      // 0 => normal
    this->declare_parameter<double>("range", 10.0);    // m
    this->declare_parameter<double>("gain", 20.0);     // %
    this->declare_parameter<double>("speed_of_sound", 0.0);  // m/s
    this->declare_parameter<double>("salinity", 0.0);  // ppm
    this->declare_parameter<std::string>("frame", "sonar");
    this->declare_parameter<std::string>("water", "fresh");
    this->declare_parameter<int>("colormap", 2);       // For viewer
    this->declare_parameter<bool>("raw", false);       // For viewer

    // Set salinity based on water parameter
    std::string water = this->get_parameter("water").as_string();
    if (water == "fresh") {
      this->set_parameter(rclcpp::Parameter("salinity", 0.0));
    } else {
      this->set_parameter(rclcpp::Parameter("salinity", 35.0));
    }

    // Create publisher
    ping_pub_ = this->create_publisher<sonar_oculus::msg::OculusPing>("~/ping", 1);

    // Set up parameter callback for dynamic reconfigure
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SonarOculusNode::parameters_callback, this, std::placeholders::_1));

    // Initialize networking
    if (!initialize_network()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize network");
      rclcpp::shutdown();
      return;
    }

    // Setup Sonar and messages
    sonar.m_readData.m_pSocket = &sock_tcp_;
    sonar.Connect();

    RCLCPP_INFO(this->get_logger(), "Connected!");

    // Send initial ping
    fire_sonar();

    // Create timer for main loop (50Hz)
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&SonarOculusNode::timer_callback, this));
  }

  ~SonarOculusNode()
  {
    sonar.Disconnect();
    if (sock_udp_ >= 0) close(sock_udp_);
    if (sock_tcp_ >= 0) close(sock_tcp_);
  }

private:
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool should_fire = false;
    for (const auto &param : parameters) {
      if (param.get_name() == "mode" || param.get_name() == "ping_rate" ||
          param.get_name() == "range" || param.get_name() == "gain" ||
          param.get_name() == "salinity" || param.get_name() == "speed_of_sound") {
        should_fire = true;
      }
    }

    if (should_fire && sonar.IsOpen()) {
      fire_sonar();
    }

    return result;
  }

  void fire_sonar()
  {
    int mode = this->get_parameter("mode").as_int();
    int ping_rate = this->get_parameter("ping_rate").as_int();
    double range = this->get_parameter("range").as_double();
    double gain = this->get_parameter("gain").as_double();
    double speed_of_sound = this->get_parameter("speed_of_sound").as_double();
    double salinity = this->get_parameter("salinity").as_double();

    // Clamp range based on part number
    if (part_number_ == OculusPartNumberType::partNumberM750d) {
      if (mode == 1)
        range = std::max(0.1, std::min(range, 120.0));
      else if (mode == 2)
        range = std::max(0.1, std::min(range, 40.0));
    }
    if (part_number_ == OculusPartNumberType::partNumberM1200d) {
      range = std::max(0.1, std::min(range, 30.0));
    }

    sonar.Fire(mode, ping_rate, range, gain, speed_of_sound, salinity);
  }

  bool initialize_network()
  {
    RCLCPP_INFO(this->get_logger(), "Initializing...");

    // Variable declarations
    struct sockaddr_in server_udp;
    socklen_t length_server_udp;
    int buf_size = DATALEN;
    int keepalive = 1;

    // Clear and initialize values of server network info
    length_server_udp = sizeof(server_udp);
    bzero((char *)&server_udp, length_server_udp);
    server_udp.sin_family = AF_INET;
    server_udp.sin_addr.s_addr = htonl(INADDR_ANY);
    server_udp.sin_port = htons(PORT_UDP);

    RCLCPP_INFO(this->get_logger(), "Connecting...");

    // Create the UDP listening socket
    sock_udp_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_udp_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error opening UDP listening socket");
      return false;
    }

    // Bind the UDP socket
    if (bind(sock_udp_, (struct sockaddr *)&server_udp, length_server_udp) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error binding UDP listening socket");
      return false;
    }
    listen(sock_udp_, 5);

    // Wait for sonar status message
    while (rclcpp::ok()) {
      int64_t bytes_available = 0;
      if (ioctl(sock_udp_, FIONREAD, &bytes_available) < 0) {
        RCLCPP_WARN(this->get_logger(), "ioctl failed: %s", strerror(errno));
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      if (bytes_available > 0) {
        OculusStatusMsg osm;
        unsigned bytes_read = read(sock_udp_, (char*)&osm, bytes_available);
        (void)bytes_read;

        uint32_t ts = (osm.status >> 14) & 0x0003;
        if (ts == 0) {
          RCLCPP_INFO(this->get_logger(), "Temperature OK; the sonar will ping normally");
        } else if (ts == 1) {
          RCLCPP_WARN(this->get_logger(), "Temperature high; the sonar will ping at reduced rate");
        } else if (ts == 3) {
          RCLCPP_ERROR(this->get_logger(), "Temperature shutdown; the sonar will no longer ping");
          return false;
        }

        struct in_addr ip_addr;
        ip_addr.s_addr = osm.ipAddr;
        part_number_ = osm.partNumber;
        RCLCPP_INFO(this->get_logger(), "The IP address is %s", inet_ntoa(ip_addr));
        RCLCPP_INFO(this->get_logger(), "Oculus part number is %d", part_number_);

        // Setup TCP connection
        struct sockaddr_in server_tcp;
        socklen_t length_server_tcp = sizeof(server_tcp);
        bzero((char *)&server_tcp, length_server_tcp);
        server_tcp.sin_family = AF_INET;
        server_tcp.sin_addr.s_addr = osm.ipAddr;
        server_tcp.sin_port = htons(PORT_TCP);

        // Create TCP socket
        sock_tcp_ = socket(AF_INET, SOCK_STREAM, 0);
        if (sock_tcp_ < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error opening TCP main socket");
          return false;
        }

        // Connect to the sonar
        if (connect(sock_tcp_, (struct sockaddr *)&server_tcp, length_server_tcp) < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error connecting TCP socket");
          return false;
        }

        if (setsockopt(sock_tcp_, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(buf_size)) < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error increasing RCVBUF for TCP socket");
          return false;
        }

        if (setsockopt(sock_tcp_, SOL_SOCKET, SO_KEEPALIVE, &keepalive, sizeof(keepalive)) < 0) {
          RCLCPP_ERROR(this->get_logger(), "Error setting keep alive option for TCP socket");
          return false;
        }

        listen(sock_tcp_, 5);
        return true;
      }

      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return false;
  }

  void timer_callback()
  {
    // Run the read thread sonar
    sonar.m_readData.run();

    // Get bins and beams
    unsigned int nbins = sonar.m_readData.m_osBuffer[0].m_rfm.nRanges;
    unsigned int nbeams = sonar.m_readData.m_osBuffer[0].m_rfm.nBeams;
    unsigned int id = sonar.m_readData.m_osBuffer[0].m_rfm.pingId;

    // Create message from sonar data
    if (nbeams > 0 && nbins > 0 && id > latest_id_) {
      latest_id_ = id;

      if (sonar.m_readData.m_osBuffer[0].m_rawSize) {
        std::string frame_str = this->get_parameter("frame").as_string();

        // sonar image
        sensor_msgs::msg::Image sonar_image;
        sonar_image.header.stamp = this->now();
        sonar_image.height = nbins;
        sonar_image.width = nbeams;
        sonar_image.encoding = "8UC1";
        sonar_image.step = nbeams;
        sonar_image.data.resize(nbeams * nbins);
        std::copy(sonar.m_readData.m_osBuffer[0].m_pImage,
                  sonar.m_readData.m_osBuffer[0].m_pImage + nbins * nbeams,
                  sonar_image.data.begin());

        // fire msg
        sonar_oculus::msg::OculusFire fire_msg;
        fire_msg.header.stamp = this->now();
        fire_msg.mode = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.masterMode;
        fire_msg.gamma = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gammaCorrection;
        fire_msg.flags = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.flags;
        fire_msg.range = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.range;
        fire_msg.gain = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.gainPercent;
        fire_msg.speed_of_sound = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.speedOfSound;
        fire_msg.salinity = sonar.m_readData.m_osBuffer[0].m_rfm.fireMessage.salinity;

        // sonar ping
        sonar_oculus::msg::OculusPing ping_msg;
        ping_msg.header.frame_id = frame_str;
        ping_msg.header.stamp = fire_msg.header.stamp;
        ping_msg.ping = sonar_image;
        ping_msg.fire_msg = fire_msg;
        ping_msg.ping_id = id;
        ping_msg.part_number = part_number_;

        ping_msg.start_time = sonar.m_readData.m_osBuffer[0].m_rfm.pingStartTime;
        ping_msg.bearings.resize(nbeams);
        for (unsigned int i = 0; i < nbeams; ++i)
          ping_msg.bearings[i] = sonar.m_readData.m_osBuffer[0].m_pBrgs[i];
        ping_msg.range_resolution = sonar.m_readData.m_osBuffer[0].m_rfm.rangeResolution;
        ping_msg.num_ranges = nbins;
        ping_msg.num_beams = nbeams;

        ping_pub_->publish(ping_msg);
      }
    }
  }

  // Member variables
  rclcpp::Publisher<sonar_oculus::msg::OculusPing>::SharedPtr ping_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  
  unsigned int latest_id_;
  uint16_t part_number_;
  int sock_udp_ = -1;
  int sock_tcp_ = -1;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SonarOculusNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
