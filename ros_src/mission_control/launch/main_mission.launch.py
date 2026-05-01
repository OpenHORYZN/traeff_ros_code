from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('realsense2_camera'), '/launch/rs_launch.py'
        ]),
        launch_arguments={
            'depth_module.enable':     'false',
            'pointcloud.enable':       'false',
            'rgb_camera.color_width':  '1920',
            'rgb_camera.color_height': '1080',
            'rgb_camera.color_fps':    '30',
        }.items()
    )
    
    mission_control_node = Node(
        package='mission_control',
        executable='main_mission',
        name='main_mission',
        output='screen',
        parameters=[{
            # Topics
            "odometry_topic":    "/mavros/local_position/odom",
            "image_topic":       "/world/lawn/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
            "camera_info_topic": "/world/lawn/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info",
            "twist_topic":       "/mavros/setpoint_velocity/cmd_vel",
            # Vision — marker_size in metres
            "marker_size": 0.5,
            # PID X
            "kp_x": 0.4,
            "ki_x": 0.15,
            "kd_x": 0.01,
            # PID Y
            "kp_y": 0.4,
            "ki_y": 0.15,
            "kd_y": 0.01,
            # Flight
            "down_speed":   -0.2,
            "up_speed": 1.0,
            "flight_height": 3.3,
            "camera_yaw_deg": 0.0,
            "aruco_tolerance": 0.2
        }]
    )

    return LaunchDescription([
        #realsense_launch,
        mission_control_node,
    ])