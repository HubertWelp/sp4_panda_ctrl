#!/bin/bash


# Analyse whether an IP-adress is provided as an argument for SP4Koordinator and ros_bridge
if [ -z "$1" ]; then
	IP_ADRESS_BILDANALYSATOR='127.0.0.1'
else
	IP_ADRESS_BILDANALYSATOR=$1
fi
IP_ADRESS_CHATBOT_ORCHESTRATOR=$IP_ADRESS_BILDANALYSATOR
echo $IP_ADRESS_CHATBOT_ORCHESTRATOR

# Set up permissions for USB port
sudo chmod a+rw /dev/ttyACM0

# Launch all components in tabs
gnome-terminal \
    --tab --title="ROS Core" \
        --command="bash -c 'echo -e \"\e[1;34m===== ROS Core =====\e[0m\"; source ~/catkin_ws/devel/setup.bash; roscore; exec bash'" \
    --tab --title="ROS Bridge" \
        --command="bash -c 'echo -e \"\e[1;34m===== ROS Bridge --IP $IP_ADRESS_CHATBOT_ORCHESTRATOR =====\e[0m\"; sleep 3; source ~/catkin_ws/devel/setup.bash; rosrun chatbot_lm ros_bridge.py --IP $IP_ADRESS_CHATBOT_ORCHESTRATOR; exec bash'" \
    --tab --title="SP4 Koordinator" \
        --command="bash -c 'echo -e \"\e[1;34m===== SP4 Koordinator --IP $IP_ADRESS_BILDANALYSATOR =====\e[0m\"; sleep 5; source ~/catkin_ws/devel/setup.bash; rosrun SP4Koordinator SP4Koordinator --IP $IP_ADRESS_BILDANALYSATOR; exec bash'" 
    --tab --title="Panda CLI Interface" \
        --command="bash -c 'echo -e \"\e[1;34m===== PANDA CLI CONTROLER =====\e[0m\"; sleep 2; source ~/catkin_ws/devel/setup.bash; roslaunch sp3_panda_ctrl sp3_sim.launch; exec bash'" \
    --tab --title="Coordinate Translator" \
        --command="bash -c 'echo -e \"\e[1;34m===== COORDINATE TRANSLATOR =====\e[0m\"; sleep 4; source ~/catkin_ws/devel/setup.bash; rosrun sp3_panda_ctrl coordinate_translator; exec bash'"


echo -e "\e[1;32mSP4 System with Panda launched using object detector.\e[0m"
