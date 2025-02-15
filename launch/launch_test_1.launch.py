import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! Make sure you set the package name correctly !!!
    # 시뮬레이션 시간 사용 설정
    
    package_name = 'mobile_test'
    

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={'use_sim_time': 'true'}.items()

    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'mobile_robot'],
        output='screen',
        parameters=[{'use_sim_time': True}],
        )
    
    # Print the sensor data node from mobile_test pkg
    test_controller_1 = Node(
        package='mobile_test',
        executable='test_controller_1',
        name='test_controller_1',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )


    
    # Launch them all!
    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            test_controller_1,
            ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/cmd_vel'],
            output='screen'
        )
        ]
    )