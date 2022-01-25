/*****************************************************************************/
// Inlcudes
/*****************************************************************************/

#include "image_inverter_node.h"

/*****************************************************************************/
// Implementation
/*****************************************************************************/
InvertImageNode::InvertImageNode() : Node("image_inverter")
{
    rclcpp::QoS video_qos(10);
    video_qos.keep_last(10);
    video_qos.reliable();
    video_qos.durability_volatile();

    subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image", video_qos, std::bind(&InvertImageNode::imageRecvCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/inverted_image", video_qos);

    int success = XInvert_image_Initialize(&x_inv_img_, "invert_image");

	if (success == XST_DEVICE_NOT_FOUND)
	{
		RCLCPP_FATAL(this->get_logger(), "Device invert_image not found");
		
        rclcpp::shutdown();
	}

	if (success == XST_OPEN_DEVICE_FAILED)
	{
		RCLCPP_FATAL(this->get_logger(), "Open device invert_image failed");
		
        rclcpp::shutdown();
	}

    if (success != XST_SUCCESS)
    {
        RCLCPP_FATAL(this->get_logger(), "Component invert_image initialization failed ");
    
        rclcpp::shutdown();
    }        
    else{
        RCLCPP_INFO(this->get_logger(), "Component invert_image initialization successful");
    }
}

void InvertImageNode::imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    cv::Mat img_grey;
    cv::cvtColor(cv_ptr->image, img_grey, CV_BGR2GRAY);

    cv::Mat img_grey_out;

    int status = invertImage(img_grey, &img_grey_out);

    if (!status)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed inverting image");

        return;
    }

    RCLCPP_INFO(this->get_logger(), "Inverted image");

    RCLCPP_INFO(this->get_logger(), "Initing cv ptr out");
    cv_bridge::CvImage cv_img_out;

    RCLCPP_INFO(this->get_logger(), "Assigning img_grey_out to cv_ptr_out->image");

    cv_img_out.header =  msg->header;
    cv_img_out.header.set__stamp(rclcpp::Clock().now());
    cv_img_out.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    cv_img_out.image = img_grey_out;

    RCLCPP_INFO(this->get_logger(), "Publishing inverted msg");

    InvertImageNode::publishInvertedImage(cv_img_out.toImageMsg());
}

void InvertImageNode::publishInvertedImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Publishing image");
    publisher_->publish(*msg);
}

int InvertImageNode::invertImage(const cv::Mat img_grey, cv::Mat *ptr_inv_img_grey)
{
    RCLCPP_INFO(this->get_logger(), "Calling invert_image");

    if (img_grey.total() != SIZE)
    {
        RCLCPP_INFO(this->get_logger(), "Expected image of size %d, got size %d, aborting", SIZE, img_grey.total());

        return 0;
    }

    std::vector<uint8_t> img_vec_in;

    img_vec_in.assign(img_grey.data, img_grey.data + img_grey.total());

    std::vector<uint8_t> img_vec_out(SIZE);

    int success = callIP(&img_vec_in[0], &img_vec_out[0]);

    if (!success)
    {
        RCLCPP_INFO(this->get_logger(), "Unsuccessful call to IP");

        return 0;
    }

    RCLCPP_INFO(this->get_logger(), "Successful call to IP");

    *ptr_inv_img_grey = cv::Mat(cv::Size(IMAGE_Y,IMAGE_X), CV_8UC1, img_vec_out.data());

    return 1;
}

int InvertImageNode::callIP(const uint8_t *ptr_img_data_in, const uint8_t *ptr_img_data_out)
{
    int length;

    RCLCPP_INFO(this->get_logger(), "Polling for invert_image IP ready");

    while(!XInvert_image_IsReady(&x_inv_img_));

    length = XInvert_image_Write_image_in_Bytes(&x_inv_img_, 0, (char *)ptr_img_data_in, SIZE);

    if(length == SIZE)
    {
        RCLCPP_INFO(this->get_logger(), "Wrote data to invert_image IP");
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Could not write data to invert_image IP");
        
        return 0;
    }

    RCLCPP_INFO(this->get_logger(), "Polling for invert_image IP idle");

    while(!XInvert_image_IsIdle(&x_inv_img_));

    RCLCPP_INFO(this->get_logger(), "Starting invert_image IP");

    XInvert_image_Start(&x_inv_img_);

    RCLCPP_INFO(this->get_logger(), "Started invert_image IP, polling for IP done");

    while(!XInvert_image_IsDone(&x_inv_img_));

    RCLCPP_INFO(this->get_logger(), "Reading data form invert_image IP");

    length = XInvert_image_Read_image_out_Bytes(&x_inv_img_, 0, (char *)ptr_img_data_out, SIZE);

    if(length == SIZE)
    {
        RCLCPP_INFO(this->get_logger(), "Read data from invert_image IP");
    } else
    {
        RCLCPP_ERROR(this->get_logger(), "Could not read data from invert_image IP");
        
        return 0;
    }

    return 1;
}
