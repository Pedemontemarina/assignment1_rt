from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # 1) TURTLESIM NODE
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # 2) TURTLE_SPAWN (python)
        Node(
            package='assignment1_rt',
            executable='turtle_spawn',
            name='turtle_spawn',
            output='screen'
        ),

        # TERMINAL 2: ui_node (CPP)
        ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e',
                'bash -c "ros2 run assignment1_rt ui_node"'
            ],
            output='screen'
        ),

        
         # 4) NODO DISTANCE (python)
        Node(
            package='assignment1_rt',
            executable='distance_node',
            name='distance',
            output='screen'
        )
    ])
