
#include "image_storer_node.h"

ImageStorerNode::ImageStorerNode() : Node("image_storer")
{
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.reliable();
    video_qos.durability_volatile();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", video_qos, std::bind(&ImageStorerNode::imageRecvCallback, this, std::placeholders::_1));

	this->get_parameter("store_dir", store_dir_);
}

void ImageStorerNode::imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

	char save_path[100];

	sprintf(save_path, "%s/img_%u.png", store_dir_, (int)(rclcpp::Clock().now().nanoseconds()));

	cv::imwrite(save_path, cv_ptr->image);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageStorerNode>());
    rclcpp::shutdown();

    return 0;
}