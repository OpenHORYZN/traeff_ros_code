from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    foto_period_arg        = DeclareLaunchArgument('foto_period',        default_value='15.0')
    image_input_topic_arg  = DeclareLaunchArgument('image_input_topic',  default_value='/camera/camera/color/image_raw')
    image_output_topic_arg = DeclareLaunchArgument('image_output_topic', default_value='/snapshot/image_raw')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'),
            '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'depth_module.enable':     'false',
            'pointcloud.enable':       'false',
            'rgb_camera.color_width':  '1920',
            'rgb_camera.color_height': '1080',
            'rgb_camera.color_fps':    '30',
        }.items()
    )

    manual_node = Node(
        package='mission_control',
        executable='manual_node',
        name='ManualNode',
        parameters=[{
            'foto_period':        LaunchConfiguration('foto_period'),
            'image_input_topic':  LaunchConfiguration('image_input_topic'),
            'image_output_topic': LaunchConfiguration('image_output_topic'),
        }],
        output='screen',
    )

    return LaunchDescription([
        foto_period_arg,
        image_input_topic_arg,
        image_output_topic_arg,
        realsense_launch,
        manual_node,
    ])