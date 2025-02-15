pkg name = "mobile_test"

command : 

ros2 launch mobile_test launch_robot.launch.py

ros2 topic echo /odom

ros2 topic echo /imu_plugin/out



launch file: 

rsp.launch.py, 

launch_robot.launch.py

descirption: 

  gazebo_control.xacro: differential driver controller and imu_plugin imported,
  
  inertial_macros.xacro: each inertials of link
  
  mobile_robot_core.xacro: urdf file of robot
  
  robot.urdf.xacro: mobile_robot_core.xacro file imported
