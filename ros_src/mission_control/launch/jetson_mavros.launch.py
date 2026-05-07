from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    DeclareLaunchArgument
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():


    gcs_ip = LaunchConfiguration('gcs_ip')

    # Build full MAVROS gcs_url automatically
    gcs_url = ['udp://@', gcs_ip, ':14550']

    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [FindPackageShare('mavros'), '/launch/px4.launch']
        ),
        launch_arguments={
            'fcu_url': '/dev/ttyACM0:921600',
           # 'gcs_url': gcs_url
        }.items()
    )

    # Wait for MAVROS node to fully start
    set_mav_frame = TimerAction(
        period=15.0,
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
#        gcs_ip_arg,
        mavros_launch,
        set_mav_frame
    ])
