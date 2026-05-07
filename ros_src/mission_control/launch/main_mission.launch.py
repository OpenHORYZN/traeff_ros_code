from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

from launch.actions import SetEnvironmentVariable

SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', 'ERROR'),

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
            "odometry_topic":    "/mavros/local_position/odom",
            "image_topic":       "/camera/camera/color/image_raw",
            "camera_info_topic": "/camera/camera/color/camera_info",
            "twist_topic":       "/mavros/setpoint_velocity/cmd_vel",
            "marker_size":       0.1,
            "kp_x":  0.4,  "ki_x":  0.15, "kd_x":  0.05,
            "kp_y":  0.4,  "ki_y":  0.15, "kd_y":  0.05,
            "down_speed":      -0.2,
            "up_speed":         1.0,
            "flight_height":    3.0,
            "camera_yaw_deg":   180.0,
            "aruco_tolerance":  0.2,
            "use_sim_time":     False,
        }]
    )

    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [FindPackageShare('mavros'), '/launch/px4.launch']
        ),
        launch_arguments={
            'fcu_url': '/dev/ttyACM0:921600',
            'log_level': 'error',
        }.items()
    )

    # t=0: mavros
    # t=5: camera + mission
    # t=30: set mav_frame param (25s after mission starts)

    delayed_camera_and_mission = TimerAction(
        period=15.0,
        actions=[
            realsense_launch,
            mission_control_node,
        ]
    )

    set_mav_frame = TimerAction(
        period=30.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'param', 'set',
                    '/mavros/setpoint_velocity',
                    'mav_frame',
                    'BODY_NED'
                ],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        mavros_launch,
        delayed_camera_and_mission,
        set_mav_frame,
    ])
