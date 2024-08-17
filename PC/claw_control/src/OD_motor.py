#!/usr/bin/env python3

"""
A python lib for controlling the OD-A8120-P1-6 motor.
- Receive and decode motor feedback messages.
- Generate the CAN message to send depend on the motor control command.

Author: Alexavier
Date: 2024-7-28
version: 0.1
"""

import time


def limit(value, min_value, max_value):
    """
    限幅函数
    """
    return max(min(value, max_value), min_value)


def linerMap(x, in_min, in_max, out_min, out_max):
    """
    线性映射函数
    x: 输入值
    in_min, in_max: 输入值范围
    out_min, out_max: 输出值范围
    retrun: 映射后的值
    """
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def float_to_uint(x, x_min, x_max, bits):
    """
    浮点数转换为bits位无符号整数
    """
    span = x_max - x_min
    offset = x_min
    x = (x - offset) / span    # 将x映射到[0, 1]范围内
    x = x * (2**bits - 1)      # 将x映射到[0, 2^bits - 1]范围内
    return round(x)            # 四舍五入并转换为整数


class MotorState:
    def __init__(self):
        self.mode = 0           # 电机模式
        self.pos = 0            # 电机位置
        self.vel = 0            # 电机速度
        self.cur = 0            # 电机电流
        self.temp = 0           # 电机温度


class MotorCtrlCmd:
    def __init__(self):
        self.pos_tar = 0        # 位置控制目标
        self.vel_tar = 0        # 速度控制目标
        self.KP = 0
        self.KD = 0
        self.cur_tar = 0        # 电流控制目标
        self.tor_ff = 0         # 力矩前馈


class ODMotor:
    AUTO_MODE = 0x01            # 自动报文模式
    ANS_MODE = 0x02             # 问答模式
    SETTING_ID = 0x7FF          # 设置指令ID

    CUR_MAX = 18                # 电机最大电流
    # 广播模式数据比例系数
    POS_RATIO = 0.01            # 位置反馈数据比例系数 -> 度
    VEL_RATIO = 0.1             # 速度反馈数据比例系数 -> rpm
    CUR_RATIO = 0.01            # 电流反馈数据比例系数 -> A
    # 问答模式数据范围（转换关系）
    POS_MIN = -12.5             # 位置最小值 -> rad
    POS_MAX = 12.5              # 位置最大值 -> rad
    VEL_MIN = -18.0             # 速度最小值 -> rad/s
    VEL_MAX = 18.0              # 速度最大值 -> rad/s
    CUR_MIN = -30.0             # 电流最小值 -> A
    CUR_MAX = 30.0              # 电流最大值 -> A
    TOR_MAX = 30.0              # 力矩最大值 -> N*m
    TOR_MIN = -30.0             # 力矩最小值 -> N*m
    KP_MAX = 500.0              # 位置环比例系数最大值
    KP_MIN = 0.0                # 位置环比例系数最小值
    KD_MAX = 50.0               # 位置环微分系数最大值
    KD_MIN = 0.0                # 位置环微分系数最小值

    def __init__(self, id, mode):
        self.id = id                # 问答模式下电机的ID
        self.broad_id = 0x204 + id  # 广播模式下接收电机的ID
        self.motor_state = MotorState()
        self.motor_ctrl_cmd = MotorCtrlCmd()
        self.motor_state.mode = mode

    def inquireCanID(self):
        """
        查询电机CAN ID
        """
        msg = bytearray([0xFF, 0xFF, 0x00, 0x82])
        return msg

    def update(self, can_id, data):
        """
        更新电机状态
        """
        # 广播模式
        if can_id == self.broad_id:
            self.motor_state.pos = (
                data[0] << 8 | data[1]) * self.POS_RATIO
            self.motor_state.vel = (
                data[2] << 8 | data[3]) * self.VEL_RATIO
            self.motor_state.cur = (
                data[4] << 8 | data[5]) * self.CUR_RATIO
        # 问答模式
        elif can_id == self.id:
            msg_type = data[0] >> 5
            err_msg = data[0] & 0x1F
            if msg_type == 0x01:
                raw_data = data[1] << 8 | data[2]
                self.motor_state.pos = linerMap(
                    raw_data, 0, 0xFFFF, self.POS_MIN, self.POS_MAX)
                raw_data = data[3] << 8 | (data[4] >> 4)
                self.motor_state.vel = linerMap(
                    raw_data, 0, 0xFFFF, self.VEL_MIN, self.VEL_MAX)
                raw_data = (data[4] & 0x0F) << 8 | data[5]
                self.motor_state.cur = linerMap(
                    raw_data, 0, 0xFFF, self.CUR_MIN, self.CUR_MAX)
                raw_data = data[6]
                self.motor_state.temp = (raw_data - 50)/2

    def packMsg(self):
        """
        打包电机控制指令
        """
        msg = bytearray()
        if self.motor_state.mode == self.AUTO_MODE:
            # 自动模式（广播模式）
            cur_send = round(self.motor_ctrl_cmd.cur_tar / self.CUR_RATIO)
            msg.append(cur_send >> 8)
            msg.append(cur_send & 0xFF)

        elif self.motor_state.mode == self.ANS_MODE:
            # 力位混合控制
            cmd_type = 0x00
            kp_send = float_to_uint(
                self.motor_ctrl_cmd.KP, self.KP_MIN, self.KP_MAX, 12)
            kd_send = float_to_uint(
                self.motor_ctrl_cmd.KD, self.KD_MIN, self.KD_MAX, 9)
            pos_send = float_to_uint(
                self.motor_ctrl_cmd.pos_tar, self.POS_MIN, self.POS_MAX, 16)
            vel_send = float_to_uint(
                self.motor_ctrl_cmd.vel_tar, self.VEL_MIN, self.VEL_MAX, 12)
            tor_send = float_to_uint(
                self.motor_ctrl_cmd.tor_ff, self.TOR_MIN, self.TOR_MAX, 12)
            msg.append((cmd_type << 5) | (kp_send >> 7))
            msg.append(((kp_send & 0x01) << 1) | (kd_send >> 8))
            msg.append(kd_send & 0xFF)
            msg.append(pos_send >> 8)
            msg.append(pos_send & 0xFF)
            msg.append(vel_send >> 4)
            msg.append(((vel_send & 0x0F) << 4) | (tor_send >> 8))
            msg.append(tor_send & 0xFF)
            # print("kp_send: ", kp_send, "kd_send: ", kd_send,
            #       "pos_send: ", pos_send, "vel_send: ", vel_send, "tor_send: ", tor_send)
            # print("msg: ", msg)
        return msg

    def setCtrlCmd(self, pos_tar, vel_tar, cur_tar, tor_ff, KP, KD):
        """
        设置电机控制指令
        """
        self.motor_ctrl_cmd.pos_tar = pos_tar
        self.motor_ctrl_cmd.vel_tar = vel_tar
        self.motor_ctrl_cmd.cur_tar = cur_tar
        self.motor_ctrl_cmd.tor_ff = tor_ff
        self.motor_ctrl_cmd.KP = KP
        self.motor_ctrl_cmd.KD = KD

    def setMode(self, mode):
        """
        设置电机模式
        """
        self.motor_state.mode = mode

    def getPos(self):
        return self.motor_state.pos


if __name__ == "__main__":
    from usb2can import USB2CAN
    can_conf = {
        'interface': 'socketcan',
        'channel': 'can0',
        'bitrate': 1000000
    }
    usb2can = USB2CAN(can_conf)
    usb2can.start()
    od = ODMotor(7, ODMotor.ANS_MODE)
    try:
        while True:
            can_msg_list = usb2can.get_data_list()
            for can_msg in can_msg_list:
                od.update(can_msg.can_id, can_msg.data)
            print("pos: ", od.motor_state.pos, "vel: ",
                  od.motor_state.vel, "temp", od.motor_state.temp)
            od.setCtrlCmd(0, 0, 0, -1, 0, 0)
            msg = od.packMsg()
            usb2can.send_std(od.id, msg)
            time.sleep(0.01)

    except KeyboardInterrupt:
        usb2can.close()
        print("Program exit")
        exit()
