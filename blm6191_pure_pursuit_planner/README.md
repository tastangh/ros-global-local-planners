# Tüm ROS node'larını durdur
rosnode kill -a

# Arka planda çalışan tüm roslaunch süreçlerini öldür
pkill -f roslaunch
pkill -f rosmaster
pkill -f gzserver
pkill -f gzclient
pkill -f rviz

# ROS loglarını temizle (isteğe bağlı)
rosclean purge -y


cd ~/robotlar_ws
source devel/setup.bash


cd ~/robotlar_ws
catkin_make
source devel/setup.bash



roslaunch turtlebot3_gazebo turtlebot3_world.launch



source ~/robotlar_ws/devel/setup.bash
roslaunch turtlebot3_slam turtlebot3_slam.launch




source ~/robotlar_ws/devel/setup.bash
roslaunch turtlebot3_navigation move_base.launch



rostopic pub /coverage_polygon geometry_msgs/Polygon "points:
- {x: 1.0, y: 1.0, z: 0.0}
- {x: 3.0, y: 1.0, z: 0.0}
- {x: 3.0, y: 3.0, z: 0.0}
- {x: 1.0, y: 3.0, z: 0.0}"