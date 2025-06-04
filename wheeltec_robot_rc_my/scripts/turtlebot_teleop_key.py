#!/usr/bin/env python3
# coding=utf-8

import os
# 设置加密启用标志, 小车默认不使用加密
ENC_ENABLE = os.getenv('ENC_ENABLE') if os.getenv('ENC_ENABLE') else False
import socket
import threading
import time
if ENC_ENABLE:
    from CryptUtils import DESdecrypt, DESencrpyt, SM4_encrypt, SM4_decrypt
import json
import re
import rospy
from geometry_msgs.msg import Twist
import time
from math import pi

HOST = "192.168.0.192"
# PORT = 40000
PORT = 30000
CODE = "utf-8"
BUF_SIZE = 1024 * 10
ENCR_METHOD = "SM4"
VEHICLE_NAME = "Vehicle-01"
RSA_PUB = ""
# 设置模拟标志, 默认为模拟环境
# SIM_ENV = False if os.getenv("SIM_ENV") else True
SIM_ENV = False
DIR_VEL = [1,0] # 表示对准y方向 [-1,0]对准x负方向
ENABLE_LOG = True  # 设置为 False 可关闭日志输出

if not ENC_ENABLE:
    ENCR_METHOD = "NON"

def log(msg):
    if ENABLE_LOG:
        print(msg)


def print_config():
    print("=== 系统配置信息 ===")
    msg = '''
drive vehicle by openTCS
        '''
    print(msg)
    print(f"ENC_ENABLE     : {ENC_ENABLE}")
    print(f"SIM_ENV        : {SIM_ENV}")
    print(f"ENCR_METHOD    : {ENCR_METHOD}")
    print(f"HOST           : {HOST}")
    print(f"PORT           : {PORT}")
    print(f"CODE           : {CODE}")
    print(f"BUF_SIZE       : {BUF_SIZE}")
    print(f"VEHICLE_NAME   : {VEHICLE_NAME}")
    print(f"RSA_PUB        : {RSA_PUB if RSA_PUB else '未设置'}")
    print(f"当前朝向 DIR_VEL: {DIR_VEL}")
    print("====================\n")

# 小车发送对象
class ReceiveEntity:
    def __init__(self, operation=None, vehicleName=None, state=None, battery=None, position=None, instruction=None,
                 instructionFeedBack=None, timestamp=None):
        self.operation = operation
        self.vehicleName = vehicleName
        self.state = state
        self.battery = battery
        self.position = position
        self.instruction = instruction
        self.instructionFeedBack = instructionFeedBack
        self.timestamp = round(timestamp if timestamp else time.time() * 1000)

    def as_json(self):
        return json.dumps(self.__dict__)

    def __str__(self):
        return str(self.__dict__)


first_point = None

# 小车接受对象
class SendEntity:
    def __init__(self):
        self.operation = None
        self.instruction = None
        self.message = None
        self.timestamp = None
        self.others = None

    def convertStepToDict(self):
        stepstr = self.instruction["step"]
        spx = re.findall(r"spx=(.*?)[,|}]", stepstr)[0]
        spy = re.findall(r"spy=(.*?)[,|}]", stepstr)[0]
        dpx = re.findall(r"dpx=(.*?)[,|}]", stepstr)[0]
        dpy = re.findall(r"dpy=(.*?)[,|}]", stepstr)[0]
        position = {
            "sourcePoint": {
                "pose": {
                    "position": {
                        "x": int(spx),
                        "y": int(spy),
                    }
                }
            },
            "destinationPoint": {
                "pose": {
                    "position": {
                        "x": int(dpx),
                        "y": int(dpy),
                    }
                }
            }
        }
        self.instruction['step'] = position

    @staticmethod
    def to_relative_position(reference_point: dict, target_point: dict) -> dict:
        ref_pos = reference_point['position']
        tgt_pos = target_point['position']

        relative_position = {
            'x': tgt_pos['x'] - ref_pos['x'],
            'y': tgt_pos['y'] - ref_pos['y'],
        }

        return {
            'position': relative_position,
        }

    @staticmethod
    def read_value(value: str):
        global first_point
        e = SendEntity()
        # log("value: " + value)
        for k, v in json.loads(value).items():
            setattr(e, k, v)
        if e.instruction and e.instruction.get("step", None):
            e.convertStepToDict()
            # first_point = first_point if first_point else e.instruction["step"]["sourcePoint"]
            # e.instruction['step']['destinationPoint']['pose'] = SendEntity.to_relative_position(first_point["pose"],
            #                                                                          e.instruction['step'][
            #                                                                              'destinationPoint']['pose'])
            # 根据源点和目标点计算移动位置
            e.instruction['step']['destinationPoint']['pose'] = SendEntity.to_relative_position(
                e.instruction['step']['sourcePoint']['pose'], e.instruction['step']['destinationPoint']['pose'])
        return e

    def __str__(self):
        return str(self.__dict__)


def sendMsg(sock: socket.socket, m: ReceiveEntity, E=True):
    m: str = m.as_json()
    if not E:
        m += '\n'
        return sock.sendall(m.encode(CODE))
    if ENCR_METHOD == "NON":
        m = m
    elif ENCR_METHOD == "DES":
        m = DESencrpyt(m)
    elif ENCR_METHOD == "RSA":
        pass
    elif ENCR_METHOD == "SM4":
        m = SM4_encrypt(m)
    m += '\n'
    sock.sendall(m.encode(CODE))


def recvMsg1(sock: socket.socket):
    '''
    存在沾包问题,遂废弃
    '''
    tname = threading.current_thread().name
    while True:
        data = sock.recv(BUF_SIZE)
        data = data.decode(CODE)
        if ENCR_METHOD == "NON":
            pass
        elif ENCR_METHOD == "DES":
            data = DESdecrypt(data)
        elif ENCR_METHOD == "RSA":
            pass
        elif ENCR_METHOD == "SM4":
            # print(f"{tname} recv: {data}")
            data = SM4_decrypt(data)
        # print(f"s{tname} : {data}")
        data = SendEntity.read_value(data)
        # 经过坐标变换的相对坐标
        if data.instruction and data.instruction['step'] and data.instruction['step']['destinationPoint']:
            dpx_r, dpy_r = data.instruction['step']['destinationPoint']['pose']['position']['x'], \
                data.instruction['step']['destinationPoint']['pose']['position']['y']
            print(f"{tname} : {data} \n dpx_r: {dpx_r} dpy_r: {dpy_r}")
            if not SIM_ENV:
                driveVehicleByCommand(data)
            continue
        log(f"{tname} : {data}")


def recvMsg(sock: socket.socket):
    tname = threading.current_thread().name
    buffer = ""
    while True:
        try:
            data = sock.recv(BUF_SIZE).decode(CODE)
            if not data:
                break  # 连接断开
            buffer += data
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if ENCR_METHOD == "NON":
                    pass
                elif ENCR_METHOD == "DES":
                    line = DESdecrypt(line)
                elif ENCR_METHOD == "RSA":
                    pass
                elif ENCR_METHOD == "SM4":
                    line = SM4_decrypt(line)
                # 处理消息
                data_obj = SendEntity.read_value(line)
                if data_obj.instruction and data_obj.instruction['step'] and data_obj.instruction['step']['destinationPoint']:
                    dpx_r = data_obj.instruction['step']['destinationPoint']['pose']['position']['x']
                    dpy_r = data_obj.instruction['step']['destinationPoint']['pose']['position']['y']
                    print(f"{tname} : {data_obj} \n dpx_r: {dpx_r} dpy_r: {dpy_r}")
                    if not SIM_ENV:
                        driveVehicleByCommand(data_obj)
                else:
                    log(f"{tname} : {data_obj}")
        except Exception as e:
            log(f"{tname} recv error: {e}")
            break



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

def turnLeft():
    global DIR_VEL
    rotate(pi / 2)
    x, y = DIR_VEL
    DIR_VEL = [-y, x]

def turnRight():
    global DIR_VEL
    rotate(-pi / 2)
    x, y = DIR_VEL
    DIR_VEL = [y, -x]

def log(msg):
    global ENABLE_LOG
    if ENABLE_LOG:
        print(msg)

def driveVehicleByCommand(command: SendEntity):
    global DIR_VEL
    scale = 1000 / 5  # 比例尺：1单位 = 1000毫米 = 1米

    # log("[INFO] 接收到指令: {}".format(command))

    dpx_r = command.instruction['step']['destinationPoint']['pose']['position']['x']
    dpy_r = command.instruction['step']['destinationPoint']['pose']['position']['y']

    dpx_r_cm, dpy_r_cm = dpx_r / scale, dpy_r / scale  # 缩放至米单位
    log(f"[INFO] 目标点: x = {dpx_r_cm:.2f} m, y = {dpy_r_cm:.2f} m")

    dx, dy = DIR_VEL
    log(f"[INFO] 当前朝向: dx = {dx}, dy = {dy}")

    if dx == 1:  # 朝 x+
        log(f"[ACTION] 前进 {dpx_r_cm:.2f} 米（x 方向）")
        forward(dpx_r_cm)
        if dpy_r_cm > 0:
            log(f"[ACTION] 左转后前进 {dpy_r_cm:.2f} 米（y+ 方向）")
            turnLeft()
            forward(dpy_r_cm)
        elif dpy_r_cm < 0:
            log(f"[ACTION] 右转后前进 {abs(dpy_r_cm):.2f} 米（y- 方向）")
            turnRight()
            forward(abs(dpy_r_cm))

    elif dx == -1:  # 朝 x-
        log(f"[ACTION] 后退 {dpx_r_cm:.2f} 米（x- 方向）")
        forward(-dpx_r_cm)
        if dpy_r_cm > 0:
            log(f"[ACTION] 右转后前进 {dpy_r_cm:.2f} 米（y+ 方向）")
            turnRight()
            forward(dpy_r_cm)
        elif dpy_r_cm < 0:
            log(f"[ACTION] 左转后前进 {abs(dpy_r_cm):.2f} 米（y- 方向）")
            turnLeft()
            forward(abs(dpy_r_cm))

    elif dy == 1:  # 朝 y+
        log(f"[ACTION] 前进 {dpy_r_cm:.2f} 米（y 方向）")
        forward(dpy_r_cm)
        if dpx_r_cm > 0:
            log(f"[ACTION] 右转后前进 {dpx_r_cm:.2f} 米（x+ 方向）")
            turnRight()
            forward(dpx_r_cm)
        elif dpx_r_cm < 0:
            log(f"[ACTION] 左转后前进 {abs(dpx_r_cm):.2f} 米（x- 方向）")
            turnLeft()
            forward(abs(dpx_r_cm))

    elif dy == -1:  # 朝 y-
        log(f"[ACTION] 后退 {dpy_r_cm:.2f} 米（y- 方向）")
        forward(-dpy_r_cm)
        if dpx_r_cm > 0:
            log(f"[ACTION] 左转后前进 {dpx_r_cm:.2f} 米（x+ 方向）")
            turnLeft()
            forward(dpx_r_cm)
        elif dpx_r_cm < 0:
            log(f"[ACTION] 右转后前进 {abs(dpx_r_cm):.2f} 米（x- 方向）")
            turnRight()
            forward(abs(dpx_r_cm))

    log("[INFO] 指令执行完成，发送完成回执")
    commandExecuted()

def commandExecuted():
    global client
    # 休息两秒执行下一条指令
    time.sleep(0.5)
    r = ReceiveEntity(operation="MOV", vehicleName=VEHICLE_NAME, instructionFeedBack="done")
    sendMsg(client, r)


if __name__ == '__main__':
    print_config()
    pub = None

    if not SIM_ENV:
        rospy.init_node('turtlebot_teleop')  # 创建ROS节点
        pub = rospy.Publisher('~cmd_vel', Twist, queue_size=5)  # 创建速度话题发布者，'~cmd_vel'='节点名/cmd_vel'

    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        client.connect((HOST, PORT))
        trecv = threading.Thread(target=recvMsg, args=(client,), name="recvMsg")
        trecv.start()
        sendMsg(client, ReceiveEntity(operation="HANDSHAKE", vehicleName=VEHICLE_NAME,
                                      instructionFeedBack=f"{ENCR_METHOD}{VEHICLE_NAME}"),
                E=False)  # 通告服务器车辆名称, 并且协商加密方式, 此消息为明文
        while True:
            # msg = input("type msg send to openTCS server: ")
            msg = input()
            r = ReceiveEntity(operation="MOV", vehicleName=VEHICLE_NAME,
                              instructionFeedBack=msg)
            if msg == "disconnect":
                sendMsg(client, r)
                break
            if msg == "exit":
                stop()
                client.close()
                raise Exception("exit...")
                break
            sendMsg(client, r)

        # trecv.join()
    except socket.error as msg:
        print(msg)
    finally:
        client.close()
    client.close()
    exit()
