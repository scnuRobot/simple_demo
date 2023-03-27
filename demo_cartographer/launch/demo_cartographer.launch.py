import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 定位到功能包的地址
    pkg_share = FindPackageShare(package='demo_cartographer').find('demo_cartographer')

    #=====================运行节点需要的配置=======================================================================
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(pkg_share, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='demo.lua')
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    
    #=====================声明节点=================================

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
        )
    
    '''
    # control
    control_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        arguments=['/model/']
    )
    '''
    # Bridge
    bridge_nodes = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/model/tugbot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
                   '/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
                   '/model/tugbot/pose@geometry_msgs/msg/Pose@ignition.msgs.Pose',
                   '/model/tugbot/tf@geometry_msgs/msg/Transform@ignition.msgs.Pose'],
        remappings=[
            ('/cmd_vel','/model/tugbot/cmd_vel'),
            ('/laser_scan','/world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan'),
        ],
        output='screen'
    )







    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    ld.add_action(bridge_nodes)
    '''
    ld.add_action(control_node)
    '''
    return ld
