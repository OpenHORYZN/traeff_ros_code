from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    mavros_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            [FindPackageShare('mavros'), '/launch/px4.launch']
        ),
        launch_arguments={
            'fcu_url': 'udp://:14540@127.0.0.1:14557'
        }.items()
    )

    # Wait for MAVROS node to fully start
    set_mav_frame = TimerAction(
        period=8.0,
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
        set_mav_frame
    ])
