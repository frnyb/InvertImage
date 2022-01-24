#ifndef IMAGE_INVERTER_H
#define IMAGE_INVERTER_H

/*****************************************************************************/
// Inlcudes
/*****************************************************************************/

#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>

extern "C" {
#include "xinvert_image.h"
};

/*****************************************************************************/
// Defines
/*****************************************************************************/

#define	IMAGE_X		480
#define IMAGE_Y		640
#define SIZE 		IMAGE_X*IMAGE_Y

/*****************************************************************************/
// Class
/*****************************************************************************/

class InvertImageNode : public rclcpp::Node
{
public:
    InvertImageNode();

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    XInvert_image inv_img_;

    void imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishInvertedImage(const sensor_msgs::msg::Image::SharedPtr msg);

    int invertImage(const sensor_msgs::msg::Image::SharedPtr msg);
};

#endif

