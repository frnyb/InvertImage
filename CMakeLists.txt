cmake_minimum_required(VERSION 3.5)
project(image-inverter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)

include_directories(include/InvertImageIP/ImageInverter/drivers/invert_image_v1_0/src)
link_directories (include/InvertImageIP/ImageInverter/drivers/invert_image_v1_0/src)    

add_executable(image_inverter 
    src/image_inverter_node.cpp
    include/InvertImageIP/ImageInverter/drivers/invert_image_v1_0/src/xinvert_image.c
    include/InvertImageIP/ImageInverter/drivers/invert_image_v1_0/src/xinvert_image_linux.c
)
ament_target_dependencies(image_inverter 
    rclcpp 
    sensor_msgs
    OpenCV
    cv_bridge
)

add_executable(image_storer
    src/image_storer_node.cpp
)
ament_target_dependencies(image_storer 
    rclcpp 
    sensor_msgs
    OpenCV
    cv_bridge
)

install(TARGETS
    image_inverter
    image_storer
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
