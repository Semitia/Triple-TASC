"""
This is a serial controller for N32 finger board,
which is equipped with 2 N20 DC motors and 2 Strain Gages.

author: Alexavier
GitHub: github.com/semitia
date: 2024-07-23
version: 0.0.1
"""

import time
import serial
import struct
import numpy as np
from N20 import N20Ctrl
from VofaRecv import VofaRecv


class N32Ctrl:
    """
    N32控制器，用于控制N32手指板上的2个N20电机和2个应变片
    """
    VOFA_MAX_CHANNELS = 10
    N20_NUM = 2
    STRAIN_GAGE_NUM = 2
    FINGER_STATE = 0

    # N20_SPD_ORDER_1 = 1
    # N20_SPD_ORDER_2 = 2
    # N20_POS_ORDER_1 = 3
    # N20_POS_ORDER_2 = 4
    # Gage_ORGER_1 = 5
    # Gage_ORDER_2 = 6

    def __init__(self, index, port, baud):
        self.index = index
        self.state = 0
        self.motor = [None] * self.N20_NUM                          # 初始化self.motor
        self.motor = [N20Ctrl(i) for i in range(self.N20_NUM)]      # 2个电机
        self.strain_gage = [0] * self.STRAIN_GAGE_NUM               # 初始化self.strain_gage
        self.vofa = VofaRecv(port, baud)
        self.vofa.start()
        print("N32Ctrl Init Complete")

    def update(self):
        recv_data = self.vofa.read_data()
        if len(recv_data) < 2*self.N20_NUM + self.STRAIN_GAGE_NUM + 1:
            return
        self.state = recv_data[self.FINGER_STATE]
        for i in range(self.N20_NUM):
            self.motor[i].speed_update(recv_data[1 + i])
            self.motor[i].pos_update(recv_data[self.N20_NUM + i + 1])
        for i in range(self.STRAIN_GAGE_NUM):
            self.strain_gage[i] = recv_data[2 * self.N20_NUM + i + 1]
        # print("id:", self.index, ", motor_vel_0:", self.motor[0].speed, ", motor_vel_1:", self.motor[1].speed, ", sensor:", self.strain_gage[0], self.strain_gage[1])
        return

    def send_msg(self):
        msg = b''
        for i in range(self.N20_NUM):
            msg += self.motor[i].pack()
        # print("Send Msg:", msg)
        self.vofa.ser.write(msg)


if __name__ == "__main__":
    n32 = N32Ctrl(0, "/dev/ttyUSB1", 1000000)
    try:
        while True:
            n32.update()
            n32.motor[0].set_speed(0)
            print("state: ", n32.state)
            print("spd0:", n32.motor[0].speed, "pos0:",
                  n32.motor[0].position, "strain0:", n32.strain_gage[0])
            print("spd1:", n32.motor[1].speed, "pos1:",
                  n32.motor[1].position, "strain1:", n32.strain_gage[1])
            n32.send_msg()
            time.sleep(0.2)

    except KeyboardInterrupt:
        n32.vofa.close()
        print("N32Ctrl Closed")
        exit(0)
