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
