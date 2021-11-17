import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

from launch import LaunchDescription  # noqa: E402
# from launch import LaunchIntrospector  # noqa: E402
# from launch import LaunchService  # noqa: E402

import launch_ros.actions  # noqa: E402


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(name="GalilController",
                                package='ctr_sim', executable='galil_node', output='screen'),
        launch_ros.actions.Node(name="MotorSetup", package='ctr_sim',
                                executable='motor_setup_node', output='screen'),
        launch_ros.actions.Node(name="KeyboardInput", package='ctr_sim',
                                executable='key_input_node', output='screen'),
    ])
