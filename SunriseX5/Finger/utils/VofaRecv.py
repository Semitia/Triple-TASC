"""
VofaRecv.py
Vofa接收端, 用于接收Vofa发送的数据

author: Alexavier
GitHub: github.com/semitia
date: 2024-07-23
version: 0.0.1
"""

import time
import serial
import struct
import threading


class VofaRecv:
    def __init__(self, port, baud, channels = 15):
        self.port = port
        self.baud = baud
        self.channels = 0
        self.mx_channels = channels
        self.rx_buf = bytearray()       # 接收缓冲区
        self.datas = []                 # 接收到的数据
        self.recvd_cnt = 0              # 接收数据总量
        self.ser = serial.Serial(port, baud, timeout=1)
        self.ReadPortThread = threading.Thread(target=self.read_port)
        self.port_running = False
        print("Init VofaRecv Complete with port:", port, "baud:", baud)

    def read_port(self):
        print("Start reading port")
        find_end_flag = False
        tidy_flag = False
        last_recv_ts = time.time()

        while self.port_running:

            if find_end_flag:
                self.channels = (len(self.rx_buf) // 4) - 1
                if self.channels >= 1:
                    # 将接收到的字节流转换为channel个float
                    format = '<{}f'.format(self.channels)
                    raw_data = self.rx_buf[:self.channels * 4]
                    self.datas = struct.unpack(format, raw_data)
                    self.recvd_cnt += 1
                    # last_recv_ts = time.time()
                    # 防止数据堆积，清除接收缓冲区
                    size = self.mx_channels * 8+8
                    if self.ser.in_waiting > size:
                        self.ser.read(size)
                self.rx_buf = bytearray()
                find_end_flag = False
                continue

            if not self.ser.in_waiting:
                # 避免设备断连或消息间隙导致的不断轮询，释放资源
                time.sleep(0.001)
                continue
            # print("in waiting:", self.ser.in_waiting)

            if tidy_flag:
                if self.ser.in_waiting < 4:
                    continue
                self.rx_buf += self.ser.read(4)
            else:
                self.rx_buf += self.ser.read(1)

            if len(self.rx_buf) <= 4:
                continue

            if self.rx_buf[-4:] != b'\x00\x00\x80\x7f':  # [0, 0, 128, 127]:
                continue
            # print("Received data:", self.rx_buf)
            find_end_flag = True
            tidy_flag = True


        print("Stopped reading port")
        return None

    def start(self):
        time.sleep(0.1)  # 增加小延时，你别说还真有用
        self.port_running = True
        self.ReadPortThread.start()

    def stop(self):
        if self.port_running:
            self.port_running = False
            self.ReadPortThread.join()  # 等待线程执行完毕

    def close(self):
        self.stop()
        if self.ser.is_open:
            self.ser.close()

    def read_data(self):
        return self.datas


# 使用示例
if __name__ == "__main__":
    vofa = VofaRecv('/dev/ttyS1', 230400)  # 根据实际情况配置端口、波特率
    vofa.start()
    try:
        while True:
            # vofa.read_port()
            floats = vofa.read_data()
            if floats:
                print("Received floats:", floats)
            time.sleep(0.3)
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        vofa.close()
