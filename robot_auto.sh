gnome-terminal --tab --title="Launch" -- bash -c "ros2 launch lbringup bringup.launch.py base_serial_port:=/dev/ttyACM0;
                                                 echo Press any key to close;
                                                 read -n 1"


gnome-terminal --tab --title="Control" -- bash -c "source install/setup.bash;
                                                   ros2 run robotRos_pkg robotRos_node;
                                                   echo Press any key to close;
                                                   read -n 1"


gnome-terminal --tab --title="Target" -- bash -c "source install/setup.bash;
                                                 ros2 run robot_pose robotRos_sendP;
                                                 echo Press any key to close;
                                                 read -n 1"





# gnome-terminal --tab --title="Robot" -- bash -c "source install/setup.bash;
#                                                   ros2 launch ado_description robot.launch.py;
#                                                   echo Press any key to close;
#                                                   read -n 1"