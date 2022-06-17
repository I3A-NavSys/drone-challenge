# Ejecutar como "$ ./round1.sh".

clear

source ros_ip_set.sh
echo '$'ROS_IP: $ROS_IP

sleep 2
roslaunch drone_ros round1.launch
