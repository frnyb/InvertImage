/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "image_inverter_node.h"

/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InvertImageNode>());
    rclcpp::shutdown();

    return 0;
}
