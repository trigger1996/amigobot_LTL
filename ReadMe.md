1  桌上的蓝色路由器是给AmigoBot用的，附带给电脑供网，这么做主要是为了让电脑在http协议的时候搜索车的IP能优先找车，而不是找别的网址

2 路由器IP: 10.0.126.1，车的IP贴在车上，路由器管理密码：l484484484

3 路由器SSID和密码不能改（没有连接密码），改了可能车连不上

___________________
#关于车的配置
http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA

只要用到rosaria即可

git clone https://github.com/amor-ros-pkg/rosaria.git

git clone http://github.com/reedhedges/AriaCoda.git

__________________
#关于车的控制

roscore

rosrun rosaria RosAria _port:=10.0.126.32:8101

rostopic pub -r 5 /RosAria/cmd_vel geometry_msgs/Twist '[0.1, 0.0, 0.0]' '[0.0, 0.0, 0.0]'

__________________
# Gazebo仿真和数据录制
cd ~/
cd catkin_ws/src/amigobot_LTL/launch/smaller_ground/
roslaunch multi_turtlebot_w_obtained_traj_sg.launch
rosbag record /amigobot_1/odom /amigobot_1/path /amigobot_1/cmd_vel  /amigobot_2/odom /amigobot_2/path /amigobot_2/cmd_vel /amigobot_3/odom /amigobot_3/path /amigobot_3/cmd_vel

# for 4 vehicles
rosbag record /amigobot_1/odom /amigobot_1/path /amigobot_1/cmd_vel  /amigobot_2/odom /amigobot_2/path /amigobot_2/cmd_vel /amigobot_3/odom /amigobot_3/path /amigobot_3/cmd_vel /amigobot_4/odom /amigobot_4/path /amigobot_4/cmd_vel

# for complex
rosbag record /amigobot_1/odom /amigobot_1/path /amigobot_1/cmd_vel  /amigobot_2/odom /amigobot_2/path /amigobot_2/cmd_vel /amigobot_3/odom /amigobot_3/path /amigobot_3/cmd_vel /amigobot_4/odom /amigobot_4/path /amigobot_4/cmd_vel /amigobot_5/odom /amigobot_5/path /amigobot_5/cmd_vel


__________________
#关于turtlebot_gazebo
很有意思
车辆在roslaunch文件内的位置会映射到odom的初始位置，改变roslaunch，会改变odom的初始位置
但是改变roslaunch文件内的角度不会改变车辆odom的初始角度，怎么转roslaunch内的yaw，车开始都会认为自身角度为0