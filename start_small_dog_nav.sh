#!/bin/bash
gnome-terminal --window -e 'bash -c "source ~/code/omni_gazebo_ws/devel/setup.bash; roslaunch omni_gazebo gazebo.launch; exec bash"' \
--tab -e 'bash -c "sleep 3; source ~/code/omni_gazebo_ws/devel/setup.bash; roslaunch omni_navigation sim_navigation.launch dog_name:=small_dog; exec bash"' \
#--tab -e 'bash -c "sleep 3; python /home/nuc11/code/omni_gazebo_ws/src/omni_robot/omni_gazebo/scripts/cmd_vel_timeout.py; exec bash"' \

                                                                                                    
