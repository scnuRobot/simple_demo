# bridge
## 2d 雷达

### omni lidar
```
source /opt/ros/foxy/setup.bash
ros2 run ros_ign_bridge parameter_bridge /world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan --ros-args -r /world/world_demo/model/tugbot/link/scan_omni/sensor/scan_omni/scan:=/laser_scan
```



## 可视化
```
source /opt/ros/foxy/setup.bash
rviz2
```
把fixed_frame改为，tugbot/scan_omni/scan_omni

## 控制
转成ROS的topic
```
source /opt/ros/foxy/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/tugbot/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
```
在终端控制
```
source /opt/ros/foxy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/model/tugbot/cmd_vel
```


## imu
```
source /opt/ros/foxy/setup.bash
ros2 run ros_ign_bridge parameter_bridge /imu@sensor_msgs/msg/Imu[ignition.msgs.IMU
```
	
laser 对于 base_link 的坐标
-0.1855 0 0.5318 0 0 0