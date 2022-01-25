import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    usb_cam_config = os.path.join(
        get_package_share_directory('image-inverter'),
        'config',
        "camera_params.yaml"
    )
    usb_cam = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        name="usb_cam",
        namespace="usb_cam",
        parameters=[{
                'params-file': usb_cam_config
        }]
    )

    image_inverter = Node(
        package="image-inverter",
        executable="image_inverter",
        name="image_inverter",
        namespace="image_inverter",
        remappings=[("/image", "/usb_cam/image_raw")]
    )

    img_storer_org_config = os.path.join(
        get_package_share_directory("image-inverter"),
        'config',
        'img_storer_org_params.yaml'
    )
    image_storer_org = Node(
        package="image-inverter",
        executable="image_storer",
        name="image_storer",
        namespace="image_storer_org",
        parameters=[img_storer_org_config],
        remappings=[("/image", "/image_inverter/org_image")]
    )

    img_storer_inv_config = os.path.join(
        get_package_share_directory("image-inverter"),
        'config',
        'img_storer_inv_params.yaml'
    )
    image_storer_inv = Node(
        package="image-inverter",
        executable="image_storer",
        name="image_storer",
        namespace="image_storer_inv",
        parameters=[img_storer_inv_config],
        remappings=[("/image", "/image_inverter/inv_image")]
    )

    return LaunchDescription([
        usb_cam,
        image_inverter,
        image_storer_org,
        image_storer_inv
    ])
