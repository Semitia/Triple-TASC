"""
This a library to control a N20 DC motor when more than one motor is used.
As a result, the controller need a serial port, but don't need to init and read it.

author: Leonaruic
GitHub: github.com/semitia
date: 2023-09-04
version: 0.0.1
"""
import time
import numpy as np
import struct

SPD_SEND_SCALE = 1000  # 电机速度发送时的缩放比例
POS_SEND_SCALE = 1000  # 电机位置发送时的缩放比例
# 定义指令类型
SET_SPD = 0x00
SET_POS = 0x01
SET_ORIGIN = 0x03


def add_tail(cmd):
    # 在指令后面添加0x0D 0x0A作为帧尾
    cmd.extend([0x0D, 0x0A])
    return cmd


# 定义数据包结构
class MotorMsg:
    def __init__(self, cmd, data):
        self.cmd = cmd
        self.data = data


class N20Ctrl:
    def __init__(self, num):
        self.id = num  # 电机编号
        self.speed = 0  # 电机速度 rad/s
        self.position = 0  # 电机位置 rad
        self.speed_updated = False  # 电机速度是否更新
        self.pos_updated = False  # 电机位置是否更新
        self.motor_msg = MotorMsg(0, 0)  # 电机指令
        print("N20 motor Ctrl ", num, " Init Complete!")

    def pack(self):
        # 'B' for uint8 (1 byte) and 'h' for int16 (2 bytes)
        packed_msg = struct.pack('<Bh', self.motor_msg.cmd, self.motor_msg.data)
        # print("Packed msg: ", packed_msg)
        return packed_msg

    def set_speed(self, speed):
        # 更新速度指令
        self.motor_msg.cmd = SET_SPD
        self.motor_msg.data = np.int16(speed * SPD_SEND_SCALE)
        return

    def set_position(self, target_pos):
        # 更新位置指令
        self.motor_msg.cmd = SET_POS
        self.motor_msg.data = np.int16(target_pos * POS_SEND_SCALE)
        return

    def speed_update(self, speed):
        """
        更新电机速度,同时将speed_updated置为True
        :param speed:
        :return:
        """
        self.speed = speed
        self.speed_updated = True

    def pos_update(self, position):
        """
        更新电机位置,同时将pos_updated置为True
        :param position:
        :return:
        """
        self.position = position
        self.pos_updated = True

    def pos_reset(self):
        self.motor_msg.cmd = SET_ORIGIN
        self.motor_msg.data = 0
        return self.pack()
