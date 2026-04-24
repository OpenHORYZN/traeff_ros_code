from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='mission_control',
            executable='main_mission',
            name='main_mission',
            output='screen',

            parameters=[{
                # Topics
                "odometry_topic": "/mavros/local_position/odom",
                "image_topic": "/camera/image_raw",
                "camera_info_topic": "/camera/camera_info",
                "twist_topic": "/mavros/setpoint_attitude/cmd_vel",

                # Vision
                "marker_size": 0.15,

                # PID X
                "kp_x": 0.5,
                "ki_x": 0.0,
                "kd_x": 0.1,

                # PID Y
                "kp_y": 0.5,
                "ki_y": 0.0,
                "kd_y": 0.1,

                # Flight
                "down_speed": -0.2,
                "flight_height": 2.3,
            }]
        )

    ])