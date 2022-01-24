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


}

void InvertImageNode::imageRecvCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Image received");

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);

    int status = invertImage(cv_ptr->image);

    //InvertImageNode::publishInvertedImage(msg);
}

void InvertImageNode::publishInvertedImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Publishing image");
    publisher_->publish(*msg);
}

int InvertImageNode::invertImage(const cv::Mat image_bgr)
{
    RCLCPP_INFO(this->get_logger(), "Calling invert_image");

    cv::Mat greyMat;
    cv::cvtColor(image_bgr, greyMat, CV_BGR2GRAY);

    std::vector<uint8_t> img_vec;

    if(greyMat.isContinuous())
    {
        img_vec.assign(greyMat.data, greyMat.data + greyMat.total());

        RCLCPP_INFO(this->get_logger(), "Image is continuous lalal!!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Image is discont!!");

        return 0;
    }

    return 1;
}

















//int main(int argc, char * argv[])
//{
//    rclcpp::init(argc, argv);
//
//
//
//        int success;
//
//        uint32_t reply;
//
//        std::cout << "Initializing invert_image component...\n\r";
//
//        success = XInvert_image_Initialize(&x_ii, "invert_image");
//
//	if (success == XST_DEVICE_NOT_FOUND)
//	{
//		std::cout << "Device not found\n\r";
//		while(1);
//	}
//
//	if (success == XST_OPEN_DEVICE_FAILED)
//	{
//		std::cout << "Open device failed\n\r";
//		while(1);
//	}
//
//        if (success != XST_SUCCESS)
//        {
//            std::cout << "Component initialization failed!\n\r";
//            while (1);
//        } else
//        {
//            std::cout << "Component initialization successful!\n\r";
//        }
//
//        std::cout << "Creating test data\n\r";
//
//        uint8_t *data_send = (uint8_t *)malloc(SIZE);
//
//        if (data_send == NULL)
//        {
//            std::cout << "Failed allocating memory!\n\r";
//            while(1);
//        }
//
//        uint8_t counter = 1;
//        for (int i = 0; i < SIZE; i++)
//        {
//            data_send[i] = counter++;
//        }
//
//        std::cout << "Finished initializing data.\n\r";
//
//        std::cout << "First 5 entries of data:\n\r";
//        std::cout << (int)data_send[0]
//            << "\t" << (int)data_send[1]
//            << "\t" << (int)data_send[2]
//            << "\t" << (int)data_send[3]
//            << "\t" << (int)data_send[4]
//            << std::endl;
//
//        std::cout << "Last 5 entries of data:\n\r";
//        std::cout << (int)data_send[SIZE-1]
//            << "\t" << (int)data_send[SIZE-2]
//            << "\t" << (int)data_send[SIZE-3]
//            << "\t" << (int)data_send[SIZE-4]
//            << "\t" << (int)data_send[SIZE-5]
//            << std::endl;
//
//        std::cout << "Polling for invert_image ready...\n\r";
//
//        while(!XInvert_image_IsReady(&x_ii));
//
//        std::cout << "Invert image is ready.\n\r";
//
//        std::cout << "Writing image_in...\n\r";
//
//    while(true)
//    {
//        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
//
//        reply = XInvert_image_Write_image_in_Bytes(&x_ii, 0, (char *)data_send, SIZE);
//
//        std::cout << "Wrote %u bytes of data.\n\r", reply;
//
//        while(!XInvert_image_IsIdle(&x_ii));
//
//        std::cout << "Starting core...\n\r";
//
//        XInvert_image_Start(&x_ii);
//
//        std::cout << "Finished starting core.\n\r";
//
//        std::cout << "Polling for finish...\n\r";
//
//        while(!XInvert_image_IsDone(&x_ii));
//
//        std::cout << "Invert image is done!\n\r";
//
//        std::cout << "Reading image out data...\n\r";
//
//        uint8_t *image_out_ptr = (uint8_t *)malloc(SIZE);
//
//        if (image_out_ptr == NULL)
//        {
//            std::cout << "Failed allocating memory!\n\r";
//            while(1);
//        }
//
//        reply = XInvert_image_Read_image_out_Bytes(&x_ii, 0, (char *)image_out_ptr, SIZE);
//
//        std::cout << "Read %u bytes of data.\n\r", reply;
//
//        std::cout << "First 5 entries of data:\n\r";
//        std::cout << (int)image_out_ptr[0]
//            << "\t" << (int)image_out_ptr[1]
//            << "\t" << (int)image_out_ptr[2]
//            << "\t" << (int)image_out_ptr[3]
//            << "\t" << (int)image_out_ptr[4]
//            << std::endl;
//
//        std::cout << "Last 5 entries of data:\n\r";
//        std::cout << (int)image_out_ptr[SIZE-1]
//            << "\t" << (int)image_out_ptr[SIZE-2]
//            << "\t" << (int)image_out_ptr[SIZE-3]
//            << "\t" << (int)image_out_ptr[SIZE-4]
//            << "\t" << (int)image_out_ptr[SIZE-5]
//            << std::endl;
//
//    }
//}


