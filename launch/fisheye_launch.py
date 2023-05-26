"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Your camera namespace
camera_name = 'fisheye_front'
# camera_name = 'my_camera'

# Location of configuration directory
config_dir = os.path.join(get_package_share_directory('gscam2'), 'cfg')
print(config_dir)

# Parameters file
params_file = os.path.join(config_dir, 'params.yaml')
print(params_file)

# Camera calibration file
camera_config = 'file://' + os.path.join(config_dir, 'my_camera.ini')
print(camera_config)


def generate_launch_description():
    # gscam_config = 'videotestsrc pattern=snow ! video/x-raw,width=1280,height=720 ! videoconvert'
    gscam_config = 'v4l2src device=/dev/video2 ! video/x-raw,format=YUY2,width=1920,height=1280,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM),format=BGRx ! nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1280 ! videoconvert ! video/x-raw, format=BGR'

    container = ComposableNodeContainer(
        name='my_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='gscam2',
                namespace= camera_name,
                plugin='gscam2::ImageSubscriberNode',
                name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='gscam2',
                namespace= camera_name,
                plugin='gscam2::GSCamNode',
                name='image_publisher',
                parameters=[{
                    'gscam_config': gscam_config,
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
