from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # 1) TURTLESIM NODE
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),

        # 2) TURTLE_SPAWN (python)
        Node(
            package='assignment1_rt',
            executable='turtle_spawn',
            name='turtle_spawn',
            output='screen'
        ),

        # 3) UI_NODE (cpp)
        Node(
            package='assignment1_rt',
            executable='ui_node',
            name='ui_node',
            output='screen'
        ),

        # 4) NODO DISTANCE (python)
        Node(
            package='assignment1_rt',
            executable='distance_node',
            name='distance',
            output='screen'
        ),
    ])
