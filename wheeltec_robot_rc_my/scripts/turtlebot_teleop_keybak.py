#!/usr/bin/env python3
# coding=utf-8

import rospy
from geometry_msgs.msg import Twist
import time
from math import pi

msg = """
My Control Your Turtlebot!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""


speed = 0.2 #默认移动速度 m/s
turn  = 0.5   #默认转向速度 rad/s
#以字符串格式返回当前速度
def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

def setSpeed(speed, turn):
    twist = Twist()
    twist.linear.x  = speed
    twist.linear.y  = 0
    twist.linear.z  = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = turn
    pub.publish(twist)

def stop():
    setSpeed(0, 0)

def smoothMove_linear(distance_cm, step_time=0.1, max_speed=0.2):
    distance_m = abs(distance_cm) / 100.0
    direction = 1 if distance_cm >= 0 else -1

    accel_dist = min(distance_m / 3, 0.2)  # 分配1/3距离给加速减速段（最多0.2m）
    const_dist = distance_m - 2 * accel_dist

    steps_accel = int(accel_dist / (max_speed * step_time / 5))
    for i in range(1, steps_accel + 1):
        v = direction * (max_speed * i / steps_accel)
        setSpeed(v, 0)
        time.sleep(step_time)

    steps_const = int(const_dist / (max_speed * step_time))
    for _ in range(steps_const):
        setSpeed(direction * max_speed, 0)
        time.sleep(step_time)

    for i in reversed(range(1, steps_accel + 1)):
        v = direction * (max_speed * i / steps_accel)
        setSpeed(v, 0)
        time.sleep(step_time)

    stop()

def smoothRotate(angle_rad, step_time=0.1, max_turn=0.5):
    angle = abs(angle_rad)
    direction = 1 if angle_rad >= 0 else -1

    accel_angle = min(angle / 3, 0.5)  # 分配1/3角度给加减速（最多0.5rad）
    const_angle = angle - 2 * accel_angle

    steps_accel = int(accel_angle / (max_turn * step_time / 5))
    for i in range(1, steps_accel + 1):
        t = direction * (max_turn * i / steps_accel)
        setSpeed(0, t)
        time.sleep(step_time)

    steps_const = int(const_angle / (max_turn * step_time))
    for _ in range(steps_const):
        setSpeed(0, direction * max_turn)
        time.sleep(step_time)

    for i in reversed(range(1, steps_accel + 1)):
        t = direction * (max_turn * i / steps_accel)
        setSpeed(0, t)
        time.sleep(step_time)

    stop()

def smoothRotateM(angle_rad, step_time=0.05, max_turn=0.5):
    angle = abs(angle_rad)
    direction = 1 if angle_rad >= 0 else -1

    accel_ratio = 0.3  # 加速段占比
    decel_ratio = 0.3  # 减速段占比
    const_ratio = 1.0 - accel_ratio - decel_ratio

    accel_angle = angle * accel_ratio
    decel_angle = angle * decel_ratio
    const_angle = angle * const_ratio

    def run_phase(phase_angle, phase_steps, speed_func):
        moved = 0.0
        for i in range(1, phase_steps + 1):
            ang_speed = speed_func(i, phase_steps) * direction
            setSpeed(0, ang_speed)
            time.sleep(step_time)
            moved += abs(ang_speed) * step_time
            if moved >= phase_angle:
                break

    steps_accel = max(1, int(accel_angle / (max_turn * step_time / 2)))
    steps_const = max(1, int(const_angle / (max_turn * step_time)))
    steps_decel = max(1, int(decel_angle / (max_turn * step_time / 2)))

    run_phase(accel_angle, steps_accel, lambda i, n: max_turn * i / n)
    run_phase(const_angle, steps_const, lambda i, n: max_turn)
    run_phase(decel_angle, steps_decel, lambda i, n: max_turn * (n - i + 1) / n)

    stop()


def forward(cm):
    smoothMove_linear(cm)

def rotate(rad):
    smoothRotateM(rad)

flag = 1
#主函数
if __name__=="__main__":
    if flag:
        try:
            rospy.init_node('turtlebot_teleop') #创建ROS节点
            pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5) #创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'
            # print("type your control_speed,control_turn")
            test_lib()
            while True:
                move, rot = input("please type move(cm) and rot(rad): ")
                forward(move)
                rotate(rot)

        except Exception as e:
            print(e)
        finally:
            twist = Twist()
            twist.linear.x = 0;  twist.linear.y = 0;  twist.linear.z = 0
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
            pub.publish(twist)

        exit(0)


