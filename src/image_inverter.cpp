#include <string>
#include <iostream>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

extern "C" {
#include "xinvert_image.h"
};

#define	IMAGE_X		480
#define IMAGE_Y		640
#define SIZE 		IMAGE_X*IMAGE_Y

XInvert_image x_ii;

class InvertImageNode : public rclcpp::Node
{
public:
    InvertImageNode() : Node("invert_image_node")
    {
    }

private:

};

int main(int argc, char * argv[])
{
        int success;

        uint32_t reply;

        std::cout << "Initializing invert_image component...\n\r";

        success = XInvert_image_Initialize(&x_ii, "invert_image");

	if (success == XST_DEVICE_NOT_FOUND)
	{
		std::cout << "Device not found\n\r";
		while(1);
	}

	if (success == XST_OPEN_DEVICE_FAILED)
	{
		std::cout << "Open device failed\n\r";
		while(1);
	}

        if (success != XST_SUCCESS)
        {
            std::cout << "Component initialization failed!\n\r";
            while (1);
        } else
        {
            std::cout << "Component initialization successful!\n\r";
        }

        std::cout << "Creating test data\n\r";

        uint8_t *data_send = (uint8_t *)malloc(SIZE);

        if (data_send == NULL)
        {
            std::cout << "Failed allocating memory!\n\r";
            while(1);
        }

        uint8_t counter = 1;
        for (int i = 0; i < SIZE; i++)
        {
            data_send[i] = counter++;
        }

        std::cout << "Finished initializing data.\n\r";

        std::cout << "First 5 entries of data:\n\r";
        std::cout << (int)data_send[0]
            << "\t" << (int)data_send[1]
            << "\t" << (int)data_send[2]
            << "\t" << (int)data_send[3]
            << "\t" << (int)data_send[4]
            << std::endl;

        std::cout << "Last 5 entries of data:\n\r";
        std::cout << (int)data_send[SIZE-1]
            << "\t" << (int)data_send[SIZE-2]
            << "\t" << (int)data_send[SIZE-3]
            << "\t" << (int)data_send[SIZE-4]
            << "\t" << (int)data_send[SIZE-5]
            << std::endl;

        std::cout << "Polling for invert_image ready...\n\r";

        while(!XInvert_image_IsReady(&x_ii));

        std::cout << "Invert image is ready.\n\r";

        std::cout << "Writing image_in...\n\r";

    while(true)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        reply = XInvert_image_Write_image_in_Bytes(&x_ii, 0, (char *)data_send, SIZE);

        std::cout << "Wrote %u bytes of data.\n\r", reply;

        while(!XInvert_image_IsIdle(&x_ii));

        std::cout << "Starting core...\n\r";

        XInvert_image_Start(&x_ii);

        std::cout << "Finished starting core.\n\r";

        std::cout << "Polling for finish...\n\r";

        while(!XInvert_image_IsDone(&x_ii));

        std::cout << "Invert image is done!\n\r";

        std::cout << "Reading image out data...\n\r";

        uint8_t *image_out_ptr = (uint8_t *)malloc(SIZE);

        if (image_out_ptr == NULL)
        {
            std::cout << "Failed allocating memory!\n\r";
            while(1);
        }

        reply = XInvert_image_Read_image_out_Bytes(&x_ii, 0, (char *)image_out_ptr, SIZE);

        std::cout << "Read %u bytes of data.\n\r", reply;

        std::cout << "First 5 entries of data:\n\r";
        std::cout << (int)image_out_ptr[0]
            << "\t" << (int)image_out_ptr[1]
            << "\t" << (int)image_out_ptr[2]
            << "\t" << (int)image_out_ptr[3]
            << "\t" << (int)image_out_ptr[4]
            << std::endl;

        std::cout << "Last 5 entries of data:\n\r";
        std::cout << (int)image_out_ptr[SIZE-1]
            << "\t" << (int)image_out_ptr[SIZE-2]
            << "\t" << (int)image_out_ptr[SIZE-3]
            << "\t" << (int)image_out_ptr[SIZE-4]
            << "\t" << (int)image_out_ptr[SIZE-5]
            << std::endl;

    }
}


