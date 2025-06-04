turtlebot_teleop
================

Turtlebot Teleoperation implementation.
This package used to be in turtlebot_apps repository. It has been temporarily migrated into turtlebot
because it is useful for both robot(turtlebot_apps) side and user side pc(turtlebot_interactions).

1. 小车上电
2. 启动openTCS四个模块, load model -> upload model to kernel
3. ssh rosv到小车, ros1_start启动容器, ros1进入容器
4. roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch 打开运动话题
5. roslaunch wheeltec_robot_rc_my keyboard_teleop.launch 启动小车, 默认对准x轴
6. 小车开始运动观察日志

> roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch

> roslaunch wheeltec_robot_rc_my keyboard_teleop.launch
