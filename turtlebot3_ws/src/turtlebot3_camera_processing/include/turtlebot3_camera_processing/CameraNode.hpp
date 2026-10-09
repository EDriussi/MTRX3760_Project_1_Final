//-----------------------------------------------------------------------------
// CameraNode.hpp
//
// Written by SID: 540754575
//
// Edited and cleaned by SID: 510516950
// Declares CameraNode, the ROS 2 node that relays the robot's camera images.
//-----------------------------------------------------------------------------

#ifndef CAMERANODE_HPP_
#define CAMERANODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

//---CameraNode Interface------------------------------------------------------
// CameraNode subscribes to the raw camera images on /camera/image_raw and
// republishes each one unchanged on /image. The images are not used for steering.
class CameraNode : public rclcpp::Node
{
    public:
        CameraNode();
        ~CameraNode();

    private:
        // Publishes on /image
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mImagePublisher;

        // Receives on /camera/image_raw
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mImageSubscriber;

        void CameraCallback(const sensor_msgs::msg::Image::SharedPtr aMsg);
};

#endif
