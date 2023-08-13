from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
 
def generate_launch_description():

    robot_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("robo2023"), '/launch', '/launch_robot.launch.py'])
            )

    lidar_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("ydlidar_ros2_driver"), '/launch', '/ydlidar_launch.py'])
            )

    slam_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("slam_toolbox"), '/launch', '/online_async_launch.py']),
                        launch_arguments={'use_sim_time': 'false'}.items()
            )

    nav2_launch = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare("nav2_bringup"), '/launch', '/navigation_launch.py']),
                        launch_arguments={'slam_params_file':'/home/dandog/ros2_ws/src/robo2023/config/mapper_params_online_async.yaml','use_sim_time': 'false'}.items()
            )
    
    nav2_node = Node(package='robo2023',
                      node_executable='navigation_node',
                      node_name='nav_node')
    
    imgproc_node = Node(package='robo2023',
                      node_executable='imgproc_node',
                      node_name='imgproc_node')
    
    servo_node = Node(package='robo2023',
                      node_executable='servo_node',
                      node_name='servo_node')
    
    main_node = Node(package='robo2023',
                      node_executable='main_node',
                      node_name='main_node')

    return LaunchDescription([
            robot_launch,
            lidar_launch,
            slam_launch,
            nav2_launch,
            nav2_node,
            imgproc_node,
            servo_node,
            main_node
    ])