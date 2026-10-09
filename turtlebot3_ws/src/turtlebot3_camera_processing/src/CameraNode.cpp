//-----------------------------------------------------------------------------
// CameraNode.cpp
//
// Written by SID: 540754575
//
// Edited and cleaned by SID: 510516950
// Implements CameraNode and the main() that runs it as the camera_node
// executable.
//-----------------------------------------------------------------------------

#include "turtlebot3_camera_processing/CameraNode.hpp"

//---CameraNode Implementation-------------------------------------------------
CameraNode::CameraNode()
    : Node("camera_node")
{

    // /image goes out with the default reliable QoS and a queue of 10
    auto Qos = rclcpp::QoS(rclcpp::KeepLast(10));

    mImagePublisher = create_publisher<sensor_msgs::msg::Image>("/image", Qos);

    // Sensor data QoS is best effort with a short queue, so a slow callback
    // drops frames instead of falling behind the camera
    mImageSubscriber = create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&CameraNode::CameraCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Turtlebot3 camera node has been initialised");
}

CameraNode::~CameraNode()
{
    RCLCPP_INFO(get_logger(), "Turtlebot3 camera node has been terminated");
}

// Runs once per camera frame, so the INFO log prints a line for every frame
void CameraNode::CameraCallback(const sensor_msgs::msg::Image::SharedPtr aMsg)
{
    RCLCPP_INFO(get_logger(), "Received Camera Image");

    mImagePublisher->publish(*aMsg);
}

//---Main----------------------------------------------------------------------
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraNode>());
  rclcpp::shutdown();

  return 0;
}
