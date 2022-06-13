import os
import launch
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    camera_config = os.path.join(
        get_package_share_directory('space_watch'),
        'config', 'camera.yaml'
    )

    manual_node = Node(
        package='space_watch',
        namespace='',
        executable='space_watch',
        name='manual',
        remappings=[
            ('image_raw', 'webcam/image_raw')
        ]   
    )

    rqt_image_view_node = Node(
        package='rqt_image_view',
        namespace='',
        executable='rqt_image_view',
        name='camera_view'
    )

    # A normal parameter yaml file will not work with Composable Nodes.
    # https://github.com/ros2/launch_ros/issues/156
    camera_node = Node(
        package='v4l2_camera',
        namespace='webcam',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[camera_config]
    )

    #return launch.LaunchDescription([container, rqt_image_view_node, camera_node])
    return launch.LaunchDescription([manual_node, camera_node])
