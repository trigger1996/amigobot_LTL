1  桌上的蓝色路由器是给AmigoBot用的，附带给电脑供网，这么做主要是为了让电脑在http协议的时候搜索车的IP能优先找车，而不是找别的网址

2 路由器IP: 10.0.126.1，车的IP贴在车上

3 路由器SSID和密码不能改，改了可能车连不上

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
