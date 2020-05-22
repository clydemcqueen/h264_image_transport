from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Grab h264 packets from a video4linux camera and publish on /image_raw/h264
        Node(package='h264_image_transport', node_executable='h264_cam_node', output='screen',
             node_name='h264_cam_node', parameters=[{
                'input_fn': '/dev/video2',
                'fps': '30',
                'size': '800x600',
                'frame_id': 'camera_frame',
                'camera_info_path': 'info.ini',
            }]),

        # Subscribe to /image_raw/h264, decode, and republish on /repub_raw
        # All remappings are shown for clarity
        Node(package='image_transport', node_executable='republish', output='screen',
             node_name='republish_node', arguments=[
                'h264',  # Input
                'raw'  # Output
            ], remappings=[
                ('in', 'image_raw'),
                ('in/compressed', 'image_raw/compressed'),
                ('in/theora', 'image_raw/theora'),
                ('in/h264', 'image_raw/h264'),
                ('out', 'repub_raw'),
                ('out/compressed', 'repub_raw/compressed'),
                ('out/theora', 'repub_raw/theora'),
                ('out/theora', 'repub_raw/h264'),
            ]),
    ])
