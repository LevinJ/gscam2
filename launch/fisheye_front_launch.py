"""Launch a Node with parameters and remappings."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Your camera namespace
# camera_name = 'fisheye_front'
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

def create_node(camera_name, camera_id):
    gscam_config = f'v4l2src device=/dev/video{camera_id} ! video/x-raw,format=YUY2,width=1920,height=1280,framerate=30/1 ! nvvidconv ! video/x-raw(memory:NVMM),format=BGRx ! nvvidconv ! video/x-raw,format=BGRx,width=1920,height=1280 ! videoconvert ! video/x-raw, format=BGR'
    if camera_name ==   'my_camera':
        gscam_config  = 'videotestsrc pattern=spokes ! video/x-raw, format=BGRx ,width=1920,height=1080! videoconvert'
    
    
    node = Node(
        package='gscam2',
        executable='gscam_main',
        output='screen',
        name='gscam_publisher',
        namespace=camera_name,
        parameters=[
            # Some parameters from a yaml file
            # params_file,
            # A few more parameters
            {
                'gscam_config':gscam_config,
                'preroll': False,
                'use_gst_timestamps': False,
                'frame_id': camera_name,
                'shm_name': camera_name,
                'camera_name': camera_name,  # Camera Name
                'camera_freq': 25,  # Camera frequence
                'camera_info_url': camera_config,  # Camera calibration information
            },
        ],
        # Remap outputs to the correct namespace
        remappings=[
            ('/image_raw', '/' + camera_name + '/image_raw'),
            ('/camera_info', '/' + camera_name + '/camera_info'),
        ],
        # prefix=['gdbserver localhost:3000'],
    )
    return node

def generate_launch_description():

    cameras = {}
    # cameras['my_camera'] = -1
    # cameras['fisheye_front'] = 2
    cameras['left'] = 0
    cameras['right'] = 2

    node_list = []
    for camera_name, camera_id in cameras.items():
        node_list.append(create_node(camera_name, camera_id))
        print("add node")

    return LaunchDescription(node_list)
