import time
import rospy
import signal
import threading
from PID import PID
from usb2can import USB2CAN
from OD_motor import ODMotor
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray


class FingerState:
    def __init__(self):
        self.position = 0
        self.velocity = 0
        self.current = 0
        self.temperature = 0
        self.error_code = 0
        self.mode = 0

class TP_STATE:
    ADC_MIN = 2000
    ADC_MAX = 4000    
    num2mode = ['stop', 'two', 'three_1', 'three_2', 'twone_1', 'twone_2']
    num2stick_mode = ['up', 'down', 'left', 'right', 'center']
    def __init__(self):
        self.mode = 'two'
        self.stick_mode = 'center'
        self.stick_val = [0] * 2


class ClawCtrl:
    TP_CAN_ID = 0x588
    TP_MOTOR_ID = 0x07
    TP2_CAN_ID = 0x100

    def __init__(self, can_port, baudrate):
        can_config = {
            'interface': 'socketcan',
            'channel': can_port,
            'bitrate': baudrate
        }
        self.usb2can = USB2CAN(can_config)
        self.usb2can.start()
        self.tp_cmd = 0         # teach pendant command, used for tp1
        self.tp_state = TP_STATE()  # teach pendant state， used for tp2
        self.force_feedback = 0
        self.last_force_ts = 0
        self.finger_state = FingerState()
        self.motor = ODMotor(self.TP_MOTOR_ID, ODMotor.ANS_MODE)
        self.gripper_dis = 0        # 二指夹爪直线距离
        self.motor_init_pos = 0
        self.motor_pid = PID(3, 0.0, 0.0, 6, 0, 100)
        self.pos_limit_flag = False
        self.last_static_ts = 0     # 最后一次大电流时间戳
        self.motor_ctrl_thread = threading.Thread(target=self.motor_ctrl) # 电机控制线程
        self.running = True
        self.mode_stick_mapping = {
            'two': {
                'up': 1,
                'down': 2,
                'left': 3,
                'right': 4,
                'center': 18
            },
            'three_1': {
                'up': 5,
                'down': 6,
                'left': 7,
                'right': 8,
                'center': 19
            },
            'three_2': {
                'up': 19,  # 未定义先归到18
                'down': 19, 
                'left': 19, 
                'right': 19, 
                'center': 19
            },
            'twone_1': {
                'up': 21,  # 未定义先归到18
                'down': 21, 
                'left': 13,
                'right': 14,
                'center': 21
            },
            'twone_2': {
                'up': 9,
                'down': 10, 
                'left': 11,
                'right': 12,
                'center': 20
            },
            'stop': {
                'up': 17,
                'down': 17,
                'left': 17,
                'right': 17,
                'center': 17  # stop mode 对应 17
            }
        }

    def servo_callback(self, msg):
        """
        直接给力反馈
        """
        # print("force feedback: ", msg.data)
        self.last_force_ts = time.time()
        self.force_feedback = msg.data * 10
        return

    def finger_callback(self, msg):
        self.finger_state.position = msg.data[0]
        return

    def recvCanMsg(self):
        """
        CAN 消息统一接收、分发处理
        """
        can_msg_list = self.usb2can.get_data_list()
        for can_msg in can_msg_list:
            # print('CAN : id', can_msg.can_id, 'data', can_msg.data)
            if can_msg.can_id == self.TP_CAN_ID:
                self.tp_cmd = can_msg.data[0]
            elif can_msg.can_id == self.TP_MOTOR_ID:
                self.motor.update(can_msg.can_id, can_msg.data)
                # theta * r(cm) = l
                self.gripper_dis = (self.motor.getPos() -
                                    self.motor_init_pos) * -1.8
                # print("motor pos: ", self.motor.getPos(), "dis: ", self.gripper_dis)
            if can_msg.can_id == self.TP2_CAN_ID:
                self.tp_state.mode = self.tp_state.num2mode[can_msg.data[0]]
                self.tp_state.stick_mode = self.tp_state.num2stick_mode[can_msg.data[1]]
                self.tp_state.stick_val[0] = (can_msg.data[2] << 4) + (can_msg.data[3]&0xD0)>>4
                self.tp_state.stick_val[1] = ((can_msg.data[3]&0x0F) << 8) + can_msg.data[4]
                self.tp_cmd = self.mode_stick_mapping[self.tp_state.mode][self.tp_state.stick_mode]
                print("tp_cmd:", self.tp_cmd, ", mode:", self.tp_state.mode, ", stick:", self.tp_state.stick_mode, ", value:", self.tp_state.stick_val)
                self.tp_cmd = self.mode_stick_mapping[self.tp_state.mode][self.tp_state.stick_mode]
        return

    def motor_init(self):
        """
        电机初始位置标定
        """
        init_finished = False
        last_pos = 0
        while init_finished < 50:
            self.motor.setCtrlCmd(0, 0, 0, 1, 0, 0)
            msg = self.motor.packMsg()
            self.usb2can.send_std(self.TP_MOTOR_ID, msg)
            time.sleep(0.01)

            msg_list = self.usb2can.get_data_list()
            for can_msg in msg_list:
                if can_msg.can_id == self.TP_MOTOR_ID:
                    self.motor.update(can_msg.can_id, can_msg.data)
            new_pos = self.motor.motor_state.pos

            if abs(new_pos - last_pos) < 0.01:
                init_finished += 1
            else:
                init_finished = 0
            last_pos = new_pos
            print("new pos:", new_pos)

        self.motor_init_pos = last_pos
        print("motor init pos: ", last_pos)

    def motor_ctrl(self):
        print("motor control thread start")

        while self.running:
            self.recvCanMsg()

            # 0.5s 内没有力反馈，清零
            if time.time() - self.last_force_ts > 0.5:
                self.force_feedback = 0
            # if self.force_feedback > 0:
            #     self.force_feedback = 0

            # 持续给力模式
            # print("force feedback: ", self.force_feedback)
            # self.motor.setCtrlCmd(0, 0, 0, self.force_feedback, 0, 0)

            # 位置限制模式
            if abs(self.force_feedback) > 2:    # 开始堵转，位置限制
                self.last_static_ts = time.time()
                if not self.pos_limit_flag:
                    self.pos_limit_flag = True
                    self.motor_pid.set_target(self.gripper_dis-0.1) # 0.4cm抵消回弹
            elif abs(self.force_feedback) < 0.5:
                if time.time() - self.last_static_ts > 0.4:    # 连续0.4s为0才退出限制，避免抖动
                    self.pos_limit_flag = False
                    self.motor_pid.ITerm = 0

            if self.pos_limit_flag:
                self.motor_pid.cal_output(self.gripper_dis)
                # 单向限制
                if -self.motor_pid.output < 0:
                    output = -self.motor_pid.output
                else:
                    output = 0
                self.motor.setCtrlCmd(0, 0, 0, output, 0, 0)
            else :
                self.motor.setCtrlCmd(0, 0, 0, 0, 0, 0)
            
            # print("Flag:", self.pos_limit_flag, ", tar_pos:", self.motor_pid.SetPoint, ", pos:", self.gripper_dis, ", output:", self.motor_pid.output, ", force:", self.force_feedback)
            msg = self.motor.packMsg()
            self.usb2can.send_std(self.TP_MOTOR_ID, msg)
            time.sleep(0.002)


def shutdown_hook():
    # 打印消息统计信息
    print("Message statistics:")
    for can_id, stat in claw_ctrl.usb2can.msg_stats.stats.items():
        print(
            f"ID: {can_id}, Counts: {stat['cnts']}, Frequency: {stat['freq']} Hz")

    claw_ctrl.usb2can.close()
    print("Program exit")

def signal_handler(sig, frame):
    rospy.signal_shutdown('Ctrl+C pressed')


if __name__ == "__main__":
    can_channel = rospy.get_param("can_channel", default="can2")
    baudrate = rospy.get_param("can_baudrate", default=1000000)
    claw_ctrl = ClawCtrl(can_channel, baudrate)
    claw_ctrl.motor_init()

    rospy.init_node("claw_ctrl")
    rospy.on_shutdown(shutdown_hook)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    tp_pub = rospy.Publisher("/control/hand/teach_pendant", Int32, queue_size=10)
    dis_pub = rospy.Publisher("/control/hand/distance", Float32, queue_size=10)
    rospy.Subscriber("/control/hand/servo_force", Float32, claw_ctrl.servo_callback)
    # rospy.Subscriber("/control/hand/finger", Float32MultiArray, claw_ctrl.finger_callback)
    loop_rate = rospy.Rate(30)
    claw_ctrl.motor_ctrl_thread.start()
    try:
        while not rospy.is_shutdown():
            tp_pub.publish(Int32(claw_ctrl.tp_cmd))
            dis_pub.publish(claw_ctrl.gripper_dis)
            loop_rate.sleep()
    except rospy.ROSInterruptException:
        claw_ctrl.running = False
        claw_ctrl.usb2can.close()
        print("ros interrupt")
