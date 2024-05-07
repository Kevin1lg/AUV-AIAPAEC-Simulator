from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_pkg_aplication',
            executable='graphics_data',
            name='graphics_data',
            output='screen',
            parameters=[
                {'step': 0.01}, #100 Hz
                {'time mode': 'simulation'}     # Choose between two options:
                                                # 'simulation': simulation time (recomended)
                                                # 'real': real time
            ],
        ),

        TimerAction(
            # Delay , The simulator delay is 1.5 sec aprox(consider its calculus)
            # The graphics will be actualizated to 2.5 sec aprox, consider 2.5 + 1.5 = 4 sec
            period=4.0,  #Total Delay
            actions=[
                Node(
                    package='ros_pkg_aplication',
                    executable='control_odometry_data',
                    name='control_odometry_data',
                    output='screen',
                    parameters=[
                        {'ur': 1.0},  #1.0 m/s
                        {'vr': 0.0},  #0.0 m/s
                        {'rr': 0.175},  #0.175 rad/s
                    ]
                ),
            ],
        ),
    ])