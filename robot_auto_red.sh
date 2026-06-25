gnome-terminal --tab --title="Launch" -- bash -c "ros2 launch lbringup bringup.launch.py base_serial_port:=/dev/ttyACM0;
                                                 echo Press any key to close;
                                                 read -n 1"

# sleep 5
# ===================Pake RTAB-Map===================
# gnome-terminal --tab --title="Launch" -- bash -c "ros2 launch lbringup bringup.launch.py base_serial_port:=/dev/ttyACM0 run_rtabmap:=true;
#                                                  echo Press any key to close;
#                                                  read -n 1"

gnome-terminal --tab --title="Control" -- bash -c "source install/setup.bash;
                                                   ros2 run robotRos_pkg robotRos_node;
                                                   echo Press any key to close;
                                                   read -n 1"

# sleep 


gnome-terminal --tab --title="Target" -- bash -c "source install/setup.bash;
                                                 ros2 run robot_pose robotRos_sendP;
                                                 echo Press any key to close;
                                                 read -n 1"

# sleep 1

# gnome-terminal --tab --title="Camera" -- bash -c "source install/setup.bash;
#                                                 ros2 run mehua_pkg mehua_cam;
#                                                 echo Press any key to close;
#                                                 read -n 1"

# sleep 1

# gnome-terminal --tab --title="Gui KFS" -- bash -c "source install/setup.bash;
#                                                 ros2 run gui_kfs_pkg gui_kfs;
#                                                 echo Press any key to close;
#                                                 read -n 1"

# sleep 1

gnome-terminal --tab --title="Gui + A*" -- bash -c "source install/setup.bash;
                                                ros2 run mehuaStar mehua_red;
                                                echo Press any key to close;
                                                read -n 1"

# sleep 1
# gnome-terminal --tab --title="Path Planning A*" -- bash -c "source install/setup.bash;
#                                                 ros2 run  astar_pkg astar_node;
#                                                 echo Press any key to close;
#                                                 read -n 1"

# sleep 1