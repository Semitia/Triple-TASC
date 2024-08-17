import rospy
import signal
import sys
sys.path.append('/userdata/HP09/HP09_proj/ros_ws/src/x5_claw_control/src/')
from utils.N32Ctrl import N32Ctrl
from std_msgs.msg import Int32
from std_msgs.msg import Float32MultiArray


class FingerCtrl:
    """
    通过UART控制三个手指(N32)
    """
    A = 0   # 固定手指编号
    B1 = 1  # 转动手指1
    B2 = 2  # 转动手指2
    finger_num = 3
    def __init__(self, uart_ports, baudrate):
        self.N32 = [N32Ctrl(i, uart_ports[i], baudrate) for i in range(self.finger_num)]
        self.AI = False
        self.AI_cmd_ts = 0
        self.AI_cmd = [0]*3        
        self.motor_cmd = [[0, 0], [0, 0], [0, 0]]
        self.tp_cmd = 0

    def tpCmd_callback(self, msg):
        """
        """
        self.tp_cmd = msg.data

    def update(self):
        for i in range(self.finger_num):
            self.N32[i].update()
            print("id:", i, ", motor_vel:", self.N32[i].motor[0].speed, "sensor:", self.N32[i].strain_gage[0], 
                    self.N32[i].strain_gage[1], ", msg_num:", self.N32[i].vofa.recvd_cnt)
        return

    def ai_callback(self, msg):
        self.AI = True
        self.AI_cmd_ts = rospy.get_time()
        self.AI_cmd = msg.data[-3:]

    def set_all_speeds(self, speeds):
        """
        设置所有手指的各自电机速度
        speeds: [[A1, A2], [B11, B12], [B21, B22]]
        """
        self.motor_cmd[self.A] = speeds[0]
        self.motor_cmd[self.B1] = speeds[1]
        self.motor_cmd[self.B2] = speeds[2]
        self.N32[self.A].motor[0].set_speed(speeds[0][0])
        self.N32[self.A].motor[1].set_speed(speeds[0][1])
        self.N32[self.B1].motor[0].set_speed(speeds[1][0])
        self.N32[self.B1].motor[1].set_speed(speeds[1][1])
        self.N32[self.B2].motor[0].set_speed(speeds[2][0])
        self.N32[self.B2].motor[1].set_speed(speeds[2][1])
        return

    spd = 4
    command_table = {
        1: [[spd, spd], [spd, spd], [spd, spd]],                # 二指, 履带同步使物体轴向下动
        2: [[-spd, -spd], [-spd, -spd], [-spd, -spd]],          # 二指, 履带同步使物体轴向上动
        3: [[ spd,  spd], [-spd, -spd], [-spd, -spd]],          # 二指, 履带使物体顺时针动
        4: [[-spd, -spd], [ spd,  spd], [ spd,  spd]],          # 二指, 履带使物体逆时针动
        5: [[ spd,  spd], [ spd,  spd], [ spd,  spd]],          # 三指, 履带同步使物体轴向下动
        6: [[-spd, -spd], [-spd, -spd], [-spd, -spd]],          # 三指, 履带同步使物体轴向上动
        7: [[0, 0], [0, 0], [0, 0]],                            # 三指, 使两2自由度指对称朝向1自由度指轴旋转
        8: [[0, 0], [0, 0], [0, 0]],                            # 三指, 使两2自由度指对称背离1自由度指轴旋转
        9: [[0, 0], [ spd,  spd], [ -spd,  spd]],               # 二加一指控制二指, 履带同步使物体轴向下动
        10: [[0, 0], [-spd, -spd], [spd, -spd]],                # 二加一指控制二指, 履带同步使物体轴向上动
        11: [[0, 0], [0, 0], [0, 0]],                           # 二加一指控制二指, 使两2自由度指对称朝向1自由度指轴旋转
        12: [[0, 0], [0, 0], [0, 0]],                           # 二加一指控制二指, 使两2自由度指对称背离1自由度指轴旋转
        13: [[ spd,  spd], [0, 0], [0, 0]],                     # 二加一指控制一指, 履带逆时针转
        14: [[-spd, -spd], [0, 0], [0, 0]],                     # 二加一指控制一指, 履带顺时针转
        15: [[0, 0], [0, 0], [0, 0]],                           # 无动作
        16: [[0, 0], [0, 0], [0, 0]],                           # 无动作
        17: [[0, 0], [0, 0], [0, 0]],                           # 复位
    }

    def finger_ctrl(self):
        """
        根据命令 command_table
        控制手指
        """
        if self.AI:
            if rospy.get_time() - self.AI_cmd_ts > 0.5:
                self.AI = False
                return
            speeds = [[self.AI_cmd[0], 0], [self.AI_cmd[1], 0], [self.AI_cmd[2], 0]]
        else:
            speeds = self.command_table.get(self.tp_cmd, [[0, 0], [0, 0], [0, 0]])
        # speeds = [[0,0],[0,0],[0,0]]
        # print("cmd:", self.tp_cmd, ", spd:", speeds[0][0], speeds[1][0], speeds[2][0])
        
        self.set_all_speeds(speeds)
        for i in range(self.finger_num):
            self.N32[i].send_msg()
        return

    def close(self):
        speeds = [[0, 0], [0, 0], [0, 0]]
        self.set_all_speeds(speeds)
        for i in range(self.finger_num):
            self.N32[i].send_msg()
            self.N32[i].vofa.close()
        return


def shutdown_hook():
    fingers.close()
    print("Program exit")


def signal_handler(sig, frame):
    rospy.signal_shutdown('Ctrl+C pressed')


if __name__ == "__main__":
    print(sys.executable)

    uart_ports = [rospy.get_param("uart_port0", default="/dev/ttyS2"),
                  rospy.get_param("uart_port1", default="/dev/ttyS1"),
                  rospy.get_param("uart_port2", default="/dev/ttyS3")]
    baudrate = rospy.get_param("uart_baudrate", default=230400)
    fingers = FingerCtrl(uart_ports, baudrate)

    rospy.init_node("finger_ctrl")
    rospy.on_shutdown(shutdown_hook)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    finger_motor_pub = rospy.Publisher("/control/hand/motor_read", Float32MultiArray, queue_size=10)
    finger_sensor_pub = rospy.Publisher("/control/hand/sensor", Float32MultiArray, queue_size=10)
    finger_motor_cmd_pub = rospy.Publisher("/control/hand/motor_cmd", Float32MultiArray, queue_size=10)
    rospy.Subscriber("/control/hand/teach_pendant", Int32, fingers.tpCmd_callback)
    rospy.Subscriber("/control/claw/ai_cmd", Float32MultiArray, fingers.ai_callback)
    
    loop_rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            fingers.update()
            fingers.finger_ctrl()
            msg = Float32MultiArray()
            msg.data = [fingers.N32[0].motor[0].speed, fingers.N32[1].motor[0].speed, fingers.N32[2].motor[0].speed]
            finger_motor_pub.publish(msg)
            msg = Float32MultiArray()
            msg.data = [fingers.motor_cmd[0][0], fingers.motor_cmd[1][0], fingers.motor_cmd[2][0]]
            finger_motor_cmd_pub.publish(msg)
            msg = Float32MultiArray()
            msg.data = [fingers.N32[0].strain_gage[0], fingers.N32[0].strain_gage[1], fingers.N32[1].strain_gage[0], 
                        fingers.N32[1].strain_gage[1], fingers.N32[2].strain_gage[0], fingers.N32[2].strain_gage[1]]
            finger_sensor_pub.publish(msg)
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        fingers.close()
