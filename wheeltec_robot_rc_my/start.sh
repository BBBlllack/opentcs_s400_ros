export ENC_ENABLE=False SIM_ENV=False

roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

roslaunch wheeltec_robot_rc_my keyboard_teleop.launch


# docker exec ros1 bash /home/wheeltec/wheeltec_robot/src/wheeltec_robot_rc_my/start.sh
