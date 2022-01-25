#ifndef IMAGE_STORER_NODE_H
#define IMAGE_STORER_NODE_H

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

extern "C" {
#include "xinvert_image.h"
}

/*****************************************************************************/
// Class
/*****************************************************************************/

class ImageStorerNode : public rclcpp::Node
{
public:
    ImageStorerNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;

    std::string store_dir_;

    void imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg);
};

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[]);

#endif

