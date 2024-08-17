"""
This script demonstrates how to use the python-can library to send and receive
CAN messages on a SocketCAN interface.
Provide Class USB2CAN to send and receive CAN messages.

author: Alexavier
date: 2024-7-27
version: 1.0
"""
import can
import time
import threading
import subprocess
from threading import Lock
from collections import defaultdict


def setup_can_interface(interface, bitrate):
    def is_can_interface_up():
        try:
            # 检查接口状态
            result = subprocess.run(
                ["ip", "link", "show", interface], capture_output=True, text=True, check=True)
            return "UP" in result.stdout
        except subprocess.CalledProcessError:
            return False

    if not is_can_interface_up():
        try:
            # 启动CAN接口
            subprocess.run(["sudo", "ip", "link", "set", interface,
                           "up", "type", "can", "bitrate", str(bitrate)], check=True)
            print(f"CAN interface {interface} set up with bitrate {bitrate}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to set up CAN interface: {e}")


class CanMsg:
    def __init__(self, can_id, data, data_len):
        self.can_id = can_id
        self.data = data
        self.data_len = data_len


class MsgStats:
    def __init__(self):
        # 访问 stats[can_id] 时，如果 can_id 尚不存在，defaultdict 会自动创建一个新的默认字典作为其值
        self.stats = defaultdict(lambda: {
            'cnts': 0,
            'freq': 0,
            'last_time': None
        })
        self.lock = Lock()

    def add_msg(self, can_id):
        with self.lock:
            current_time = time.time()
            stat = self.stats[can_id]

            stat['cnts'] += 1
            if stat['last_time'] is not None:
                time_diff = current_time - stat['last_time']
                if time_diff > 0:
                    stat['freq'] = 1 / time_diff
            stat['last_time'] = current_time

    def get_stats(self, can_id):
        with self.lock:
            return self.stats[can_id]


class USB2CAN:
    def __init__(self, can_config):
        setup_can_interface(can_config['channel'], can_config['bitrate'])
        self.can = can.Bus(**can_config)
        self.ReadPortThread = threading.Thread(target=self.read_port)
        self.canMsgList = []
        self.msgListLock = threading.Lock()
        self.running = False
        self.recv_freq = 100
        self.msg_stats = MsgStats()

    def read_port(self):
        while self.running:
            msg = self.can.recv()
            if msg:
                recv_msg = CanMsg(msg.arbitration_id, msg.data, len(msg.data))
                with self.msgListLock:
                    self.canMsgList.append(recv_msg)
                self.msg_stats.add_msg(msg.arbitration_id)
            # time.sleep(1/self.recv_freq)

    def start(self):
        self.running = True
        self.ReadPortThread.start()
        return

    def stop(self):
        self.running = False
        self.ReadPortThread.join()
        return

    def close(self):
        self.stop()
        self.can.shutdown()
        return

    def send_std(self, can_id, data):
        msg = can.Message(arbitration_id=can_id,
                          data=data, is_extended_id=False)
        try:
            self.can.send(msg)
            # print("Message sent")
        except can.CanError:
            print("Message NOT sent")
            return

    def get_data_list(self):
        with self.msgListLock:
            data = list(self.canMsgList)  # 返回列表的副本
            self.canMsgList.clear()  # 清空原列表
        return data


# 设置CAN接口的配置
can0_config = {
    'interface': 'socketcan',
    'channel': 'can0',
    'bitrate': 1000000
}

can1_config = {
    'interface': 'socketcan',
    'channel': 'can1',
    'bitrate': 1000000
}

# 定义要发送的消息
messages = [
    [0x01, 0x02, 0x03, 0x04, 0x01, 0x02, 0x03, 0x04],
    [0x05, 0x06, 0x07, 0x08, 0x05, 0x06, 0x07, 0x08],
    [0x09, 0x0A, 0x0B, 0x0C, 0x09, 0x0A, 0x0B, 0x0C],
    [0x0D, 0x0E, 0x0F, 0x10, 0x0D, 0x0E, 0x0F, 0x10]
]


def two_can_test():
    # 创建USB2CAN对象
    # usb2can0 = USB2CAN(can0_config)
    # usb2can1 = USB2CAN(can1_config)

    # # usb2can0.can.set_filters([
    # #     {"can_id": 0x123, "can_mask": 0x7FF, "extended": False},
    # #     {"can_id": 0x124, "can_mask": 0x7FF, "extended": False}
    # # ])
    # # usb2can1.can.set_filters([
    # #     {"can_id": 0x123, "can_mask": 0x7FF, "extended": False},
    # #     {"can_id": 0x124, "can_mask": 0x7FF, "extended": False}
    # # ])

    # # 启动CAN接口
    # usb2can0.start()
    # usb2can1.start()

    # # 发送消息
    # for i in range(2):
    #     usb2can0.send_std(0x123, messages[i])
    #     # usb2can1.send_std(0x124, messages[i])

    # # 等待接收消息
    # time.sleep(1)

    # print("can0 Received messages:")
    # msgList = usb2can0.get_data_list()
    # for msg in msgList:
    #     print(f"ID: {msg.can_id}, Data: {msg.data}")
    # print("can1 Received messages:")
    # msgList = usb2can1.get_data_list()
    # for msg in msgList:
    #     print(f"ID: {msg.can_id}, Data: {msg.data}")

    # # 关闭CAN接口
    # usb2can0.close()
    # usb2can1.close()
    return


if __name__ == "__main__":
    can_conf = {
        'interface': 'socketcan',
        'channel': 'can0',
        'bitrate': 1000000
    }
    usb2can = USB2CAN(can_conf)
    usb2can.start()
    try:
        while True:
            can_msg_list = usb2can.get_data_list()
            for can_msg in can_msg_list:
                print("ID: ", can_msg.can_id, "Data: ", can_msg
                      .data, "Data_len: ", can_msg.data_len)
            print("msg stats: ", usb2can.msg_stats.get_stats(0x588))

            time.sleep(0.01)

    except KeyboardInterrupt:
        # 打印消息统计信息
        print("Message statistics:")
        for can_id, stat in usb2can.msg_stats.stats.items():
            print(
                f"ID: {can_id}, Counts: {stat['cnts']}, Frequency: {stat['freq']} Hz")

        usb2can.close()
        print("Program exit")
        exit()
