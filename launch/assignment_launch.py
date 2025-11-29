from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # TURTLESIM NODE
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),

        # TURTLE_SPAWN 
        Node(
            package='assignment1_rt',
            executable='turtle_spawn',
            name='turtle_spawn',
            output='screen'
        ),

        # USER INTERFACE NODE 
        ExecuteProcess(
            cmd=[
                'xterm', '-hold', '-e',
                'bash -c "ros2 run assignment1_rt ui_node"'
            ],
            output='screen'
        ),

        
         # DISTANCE NODE
        Node(
            package='assignment1_rt',
            executable='distance_node',
            name='distance',
            output='screen'
        )
    ])
