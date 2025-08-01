source ~/.bashrc
gnome-terminal --window -e 'bash -c "source devel/setup.bash && roslaunch livox_ros_driver2 msg_MID360.launch; exec bash"' \
--tab -e 'bash -c "sleep 4; source devel/setup.bash && roslaunch fast_lio mapping_mid360.launch; exec bash"' \
--tab -e 'bash -c "sleep 7; source devel/setup.bash && roslaunch mavros px4.launch; exec bash"' \
--tab -e 'bash -c "sleep 8; source devel/setup.bash && roslaunch ground_station_comm ground_station.launch; exec bash"' \
--tab -e 'bash -c "source devel/setup.bash && roslaunch yolo11_ros yolo11_detection.launch; exec bash"' \