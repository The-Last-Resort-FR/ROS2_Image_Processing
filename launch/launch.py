from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('image_processor')

    return LaunchDescription([
        Node(
            package='image_processor',
            executable='image_processor',
            name='image_processor',
            parameters=[{'in_topic_left': '/lucid_cam_rgb_left'}, 
                        {'in_topic_right': '/lucid_cam_rgb_right'}, 
                        {'out_topic_left': 'lucid_cam_rgb_left_post'}, 
                        {'out_topic_right': 'lucid_cam_rgb_right_post'}],
        )
    ])