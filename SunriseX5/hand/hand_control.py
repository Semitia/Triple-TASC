import json
import rospy
import signal
from scservo_sdk import *
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import os
import sys
import tty
import termios

fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)

def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

SERVO_SPD = 1000
SERVO_DELTA_POS = 50

def limit(value, min_value, max_value):
    # 舵机方向可能是反的
    if min_value > max_value:
        min_value, max_value = max_value, min_value
    return min(max(value, min_value), max_value)

class HandCtrl:
    """
    通过UART控制5个舵机
    """
    SERVO_NUM = 5

    def __init__(self, servo_port, baudrate, servo_limit):
        self.dis = 0                                # teach pendant 夹爪距离
        self.mode = 'two'                           # 二级模式 two, three, twone, stop, trans, initial(初始态单向转出)
        self.AI = False                             # 是否由AI接管
        self.AI_cmd_ts = 0                          # 最新AI指令时间戳
        self.sub_mode = 'two'                       # 二加一指模式下的子模式，one动一指, two动二指
        self.tran_finish_mode = 'initial'           # 由于转换命令不持续，需要记录转换完成后的模式
        self.tp_cmd = 0                             # teach pendant 命令
        self.tp_cmd_list = []                       # 滤掉跳变
        self.servo_pos = [0] * self.SERVO_NUM       # 5个舵机的位置
        self.servo_spd = [0] * self.SERVO_NUM       # 5个舵机的速度
        self.servo_acc = [0] * self.SERVO_NUM       # 5个舵机的加速度
        self.servo_cur = [0] * self.SERVO_NUM       # 5个舵机的电流
        self.servo_load = [0] * self.SERVO_NUM      # 5个舵机的负载
        self.pos_cmd = [0] * self.SERVO_NUM         # 5个舵机的最终位置控制指令
        self.force_fb = 0                           # 夹爪力反馈(阻力)
        self.last_force_fb = 0                      
        self.servo_is_moving = [0] * self.SERVO_NUM
        self.last_static_ts = 0

        self.POS_LIMIT = [values for values in servo_limit.values()]
        print("servo position limitation: ", self.POS_LIMIT)
        self.pos_bias = {
            "init": [0, 0, 0, -95, 95],
            "two": [217, 212, 217, -292, 292],
            "three": [220, 220, 220, -95, 95],
            "twone": [220, 240, 240, 0, 0]
        }
        self.init_pos = [self.POS_LIMIT[i][0] + self.pos_bias["init"][i] for i in range(self.SERVO_NUM)]      # 初始位置
        self.two_pos = [self.POS_LIMIT[i][0] + self.pos_bias["two"][i] for i in range(self.SERVO_NUM)]        # 二指模式
        self.three_pos = [self.POS_LIMIT[i][0] + self.pos_bias["three"][i] for i in range(self.SERVO_NUM)]    # 三指模式
        self.twone_pos = [self.POS_LIMIT[i][0] + self.pos_bias["twone"][i] for i in range(self.SERVO_NUM)]    # 二加一指模式
        self.target_pos = [self.POS_LIMIT[i][0] + self.pos_bias["init"][i] for i in range(self.SERVO_NUM)]    # 目标位置
        self.ai_pos = [self.POS_LIMIT[i][0] + self.pos_bias["init"][i] for i in range(self.SERVO_NUM)]        # AI控制位置
        # print("init_pos: ", self.init_pos)
        # print("two_pos: ", self.two_pos)
        # print("three_pos: ", self.three_pos)
        # print("twone_pos: ", self.twone_pos)

        self.cmds = [{'type': 'pos', 'val': self.init_pos[i]} for i in range(self.SERVO_NUM)]
        self.ONE_RATIO = -22
        self.TWO_RATIO = -22
        self.THREE_RATIO = -22

        self.portHandler = PortHandler(servo_port)
        self.packetHandler = scscl(self.portHandler)
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to change the baudrate, Press any key to terminate...")
            getch()
            quit()
        # Set port baudrate
        if self.portHandler.setBaudRate(baudrate):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate, Press any key to terminate...")
            getch()
            quit()
        # for i in range(self.SERVO_NUM):
        #     self.init_pos[i] = self.read_servo_pos(i)[0]

    def write_servo(self, servo_id, position, time, spd):
        # 记录控制指令
        self.pos_cmd[servo_id] = position
        position = round(position)
        scs_comm_result, scs_error = self.packetHandler.WritePos(servo_id, position, time, spd)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))

    def read_servo_pos(self, servo_id):
        scs_present_position, scs_present_speed, scs_comm_result, scs_error = self.packetHandler.ReadPosSpeed(
            servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))        
        return scs_present_position, scs_present_speed

    def read_servo_cur(self, servo_id):
        scs_present_current, scs_comm_result, scs_error = self.packetHandler.ReadCurrent(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))        
        return scs_present_current

    def read_servo_load(self, servo_id):
        scs_present_load, scs_comm_result, scs_error = self.packetHandler.ReadLoad(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))        
        return scs_present_load 

    def read_mov_state(self, servo_id):
        moving, scs_comm_result, scs_error = self.packetHandler.ReadMoving(servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
        return moving

    def update_cmd(self, new_cmd):
        self.tp_cmd_list.append(new_cmd)
        if len(self.tp_cmd_list) > 1:
            self.tp_cmd_list.pop(0)
        same_flag = True
        raw_cmd = self.tp_cmd_list[0]
        for cmd in self.tp_cmd_list:
            if cmd != raw_cmd:
                same_flag = False
        if same_flag:
            self.tp_cmd = raw_cmd
        
    def tpCmd_callback(self, msg):
        """
        Receive teach pendant command
        """
        self.update_cmd(msg.data)
        if 15 <= self.tp_cmd <= 16:
            self.mode = 'stop'
            return

        elif self.tp_cmd == 17: # 复位
            if self.mode == 'trans':
                return

            if self.mode == 'two':
                self.target_pos = self.two_pos
            elif self.mode == 'three':
                self.target_pos = self.three_pos
            elif self.mode == 'twone':
                self.sub_mode = 'two'
                self.target_pos = self.twone_pos

            self.mode = 'trans'
            self.tran_finish_mode = 'initial'
        
        if self.mode == 'two':
            if 5 <= self.tp_cmd <= 8 or self.tp_cmd == 19:
                self.target_pos = self.three_pos
                self.mode = 'trans'
                self.tran_finish_mode = 'three'
            elif 9 <= self.tp_cmd <= 14 or 20<=self.tp_cmd<= 21:
                self.target_pos = self.twone_pos
                self.mode = 'trans'
                self.tran_finish_mode = 'twone'
                if 9 <= self.tp_cmd <= 12 or self.tp_cmd == 20:
                    self.sub_mode = 'two'
                elif 13 <= self.tp_cmd <= 14 or self.tp_cmd == 21:
                    self.sub_mode = 'one'

        elif self.mode == 'three':
            if 1 <= self.tp_cmd <= 4 or self.tp_cmd == 18:
                self.target_pos = self.two_pos
                self.mode = 'trans'
                self.tran_finish_mode = 'two'
            elif 9 <= self.tp_cmd <= 14 or 20 <= self.tp_cmd <= 21:
                self.target_pos = self.twone_pos
                self.mode = 'trans'
                self.tran_finish_mode = 'twone'
                if 9 <= self.tp_cmd <= 12 or self.tp_cmd == 20:
                    self.sub_mode = 'two'
                elif 13 <= self.tp_cmd <= 14 or self.tp_cmd == 21:
                    self.sub_mode = 'one'

        elif self.mode == 'twone':
            if 1 <= self.tp_cmd <= 4 or self.tp_cmd == 18:
                self.target_pos = self.two_pos
                self.mode = 'trans'
                self.tran_finish_mode = 'two'
            elif 5 <= self.tp_cmd <= 8 or self.tp_cmd == 19:
                self.target_pos = self.three_pos
                self.mode = 'trans'
                self.tran_finish_mode = 'three'
            elif self.tp_cmd == 20:
                self.sub_mode = "two"
            elif self.tp_cmd == 21:
                self.sub_mode = "one"

        elif self.mode == 'stop':
            if 1<= self.tp_cmd <= 4 or self.tp_cmd == 18:
                self.mode = 'trans'
                self.target_pos = self.two_pos
                self.tran_finish_mode = 'two'
            elif 5 <= self.tp_cmd <= 8 or self.tp_cmd == 19:
                self.mode = 'trans'
                self.target_pos = self.three_pos
                self.tran_finish_mode = 'three'
            elif 9 <= self.tp_cmd <= 14 or 20 <= self.tp_cmd <= 21:
                self.mode = 'trans'
                self.target_pos = self.twone_pos
                self.tran_finish_mode = 'twone'
                if 9 <= self.tp_cmd <= 12 or self.tp_cmd == 20:
                    self.sub_mode = 'two'
                elif 13 <= self.tp_cmd <= 14 or self.tp_cmd == 21:
                    self.sub_mode = 'one'
            elif 15 <= self.tp_cmd <= 16:
                self.mode = 'stop'
            # elif self.tp_cmd == 17:
            #     self.target = self.init_pos
            #     self.mode = 'trans'

        elif self.mode == 'trans':
            for i in range(self.SERVO_NUM):
                if self.servo_is_moving[i]:
                    return
            # 转换完毕
            if 1 <= self.tp_cmd <= 4 or self.tp_cmd == 18:
                self.mode = 'two'
            elif 5 <= self.tp_cmd <= 8 or self.tp_cmd == 19:
                self.mode = 'three'
            elif 9 <= self.tp_cmd <= 14 or 20 <= self.tp_cmd <= 21:
                self.mode = 'twone'
            elif 15 <= self.tp_cmd <= 16:
                self.mode = 'stop'
            elif self.tp_cmd == 17:
                self.mode = 'initial'
        
        elif self.mode == 'initial':
            if 1 <= self.tp_cmd <= 4 or self.tp_cmd == 18:
                self.mode = 'trans'
                self.target_pos = self.two_pos
                self.tran_finish_mode = 'two'
            elif 5 <= self.tp_cmd <= 8 or self.tp_cmd == 19:
                self.mode = 'trans'
                self.target_pos = self.three_pos
                self.tran_finish_mode = 'three'
            elif 9 <= self.tp_cmd <= 14 or 20 <= self.tp_cmd <= 21:
                self.mode = 'trans'
                self.target_pos = self.twone_pos
                self.tran_finish_mode = 'twone'
                if 9 <= self.tp_cmd <= 12 or self.tp_cmd == 20:
                    self.sub_mode = 'two'
                elif 13 <= self.tp_cmd <= 14 or self.tp_cmd == 21:
                    self.sub_mode = 'one'
            elif 15 <= self.tp_cmd <= 16:
                self.mode = 'stop'

    def dis_callback(self, msg):
        """
        Receive distance
        """
        self.dis = msg.data

    FORCE_RATIO = {
        'two': 0.001,
        'three': 0.001,
        'twone_one': 0.001,
        'twone_two': 0.001,
    }
    def cal_servo_force(self):
        if self.mode == 'twone':
            if self.sub_mode == 'two':
                self.force_fb = (self.servo_load[1] + self.servo_load[2])*self.FORCE_RATIO['twone_two']
            elif self.sub_mode == 'one':
                self.force_fb = self.servo_load[0]*self.FORCE_RATIO['twone_one']
        elif self.mode == 'two':
            self.force_fb = (self.servo_load[0] + self.servo_load[1] + self.servo_load[2])*self.FORCE_RATIO['two']
        elif self.mode == 'three':
            self.force_fb = (self.servo_load[0] + self.servo_load[1] + self.servo_load[2])*self.FORCE_RATIO['three']
        else :
            self.force_fb = 0
        ALPHA = 0.2
        self.force_fb = self.last_force_fb*(1-ALPHA) + self.force_fb*ALPHA
        self.last_force_fb = self.force_fb
        # 阻尼
        RES = 0.13
        if abs(self.force_fb) < RES: 
            self.force_fb = 0
        if self.force_fb > RES:
            self.force_fb -= RES
        elif self.force_fb < -RES:
            self.force_fb += RES

        # 通过加速度判断是否真堵住了
        acc_sum = 0
        spd_sum = 0
        for i in range(3):
            acc_sum += abs(self.servo_acc[i])
            spd_sum += abs(self.servo_spd[i])
        if abs(acc_sum) > 5 or spd_sum > 10:
            # 消掉堵转时的瞬动 不好使
            # now_ts = rospy.get_time()
            # print("now_ts: ", now_ts, "last_ts: ", self.last_static_ts)
            # if now_ts - self.last_static_ts > 0.33:
            #     self.force_fb = 0
            self.force_fb = 0
        else:
            self.last_static_ts = rospy.get_time()

        print("acc_sum: ", acc_sum, "force_fb: ", self.force_fb)
        return self.force_fb

    def update(self):
        for i in range(self.SERVO_NUM):
            self.servo_pos[i], new_spd = self.read_servo_pos(i)
            self.servo_cur[i] = self.read_servo_cur(i)
            self.servo_is_moving[i] = self.read_mov_state(i)
            self.servo_load[i] = self.read_servo_load(i)
            new_acc = new_spd - self.servo_spd[i]
            self.servo_acc[i] = new_acc*0.1 + self.servo_acc[i]*(1-0.1)
            self.servo_spd[i] = new_spd
            # print("servo_id: ", i, "pos: ", self.servo_pos[i], "spd: ", self.servo_spd[i], "cur: ", self.servo_cur[i], "load: ", self.servo_load[i])
        
        # # 夹爪力反馈
        # for i in range(3):
        #     force_fb = self.servo_cur[i]*0.1 - abs(self.servo_acc[i]*0.01)
        #     # print("id: ", i, "force: ", force_fb, "cur: ", self.servo_cur[i], "acc: ", self.servo_acc[i])
        #     self.force_fb += force_fb

        return

    def set_all_servo(self):
        for i in range(self.SERVO_NUM):
            if self.cmds[i]['type'] == 'pos':
                pos = self.cmds[i]['val']
                pos = limit(pos, self.POS_LIMIT[i][0], self.POS_LIMIT[i][1])
                self.write_servo(i, pos, 0, SERVO_SPD)
                # print("servo_id: ", i, "real_pos: ", self.servo_pos[i], "target_pos: ", pos, "moving: ", self.servo_is_moving[i])
            elif self.cmds[i]['type'] == 'spd':
                pos = self.servo_pos[i] + self.cmds[i]['val']
                pos = limit(pos, self.POS_LIMIT[i][0], self.POS_LIMIT[i][1])
                self.write_servo(i, pos, 0, SERVO_SPD)
                # print("servo_id: ", i, "real_pos: ", self.servo_pos[i], "target_pos: ", pos, "moving: ", self.servo_is_moving[i])
        return

    def ai_callback(self, msg):
        self.AI = True
        self.AI_cmd_ts = rospy.get_time()
        self.ai_pos = msg.data[:self.SERVO_NUM]

    def servo_ctrl(self):
        """
        根据命令控制各个舵机
        1: 二指, 履带同步使物体轴向下动------------------------黄色
        2: 二指, 履带同步使物体轴向上动
        3: 二指, 履带使物体顺时针动
        4: 二指, 履带使物体逆时针动
        5: 三指, 履带同步使物体轴向下动,-----------------------绿色
        6: 三指, 履带同步使物体轴向上动
        7: 三指, 使两2自由度指对称朝向1自由度指轴旋转
        8: 三指, 使两2自由度指对称背离1自由度指轴旋转
        9: 二加一指控制二指, 履带同步使物体轴向下动
        10: 二加一指控制二指, 履带同步使物体轴向上动，------------蓝色
        11: 二加一指控制二指, 使两2自由度指对称朝向1自由度指轴旋转
        12: 二加一指控制二指, 使两2自由度指对称背离1自由度指轴旋转
        13: 二加一指控制一指, 履带逆时针转，--------------------紫色
        14: 二加一指控制一指, 履带顺时针转
        15: 无动作
        16: 无动作
        17: 停止，白色
        18: 无动作
        """
        # print("mode: ", self.mode, ", sub_mode: ", self.sub_mode, ", tp_cmd: ", self.tp_cmd, ", target_pos", self.target_pos)

        if self.AI:
            if rospy.get_time() - self.AI_cmd_ts > 0.5:
                self.AI = False
                return
            self.cmds = [{'type': 'pos', 'val': self.ai_pos[i]} for i in range(self.SERVO_NUM)]

        # 二指
        elif self.mode == 'two':
            self.cmds = [{'type': 'pos', 'val': self.two_pos[i]} for i in range(self.SERVO_NUM)]
            # 张合控制
            delta_pos = round(self.dis * self.TWO_RATIO)
            self.cmds[0] = {'type': 'pos', 'val': self.two_pos[0] + delta_pos}
            self.cmds[1] = {'type': 'pos', 'val': self.two_pos[1] + delta_pos}
            self.cmds[2] = {'type': 'pos', 'val': self.two_pos[2] + delta_pos}

        # 三指
        elif self.mode == 'three':
            self.cmds = [{'type': 'spd', 'val': 0} for _ in range(self.SERVO_NUM)]
            # 张合控制
            delta_pos = round(self.dis * self.THREE_RATIO)
            self.cmds[0] = {'type': 'pos', 'val': self.three_pos[0] + delta_pos}
            self.cmds[1] = {'type': 'pos', 'val': self.three_pos[1] + delta_pos}
            self.cmds[2] = {'type': 'pos', 'val': self.three_pos[2] + delta_pos}

            if self.tp_cmd == 7:
                self.cmds[3] = {'type': 'spd', 'val':  SERVO_DELTA_POS}
                self.cmds[4] = {'type': 'spd', 'val': -SERVO_DELTA_POS}
            elif self.tp_cmd == 8:
                self.cmds[3] = {'type': 'spd', 'val': -SERVO_DELTA_POS}
                self.cmds[4] = {'type': 'spd', 'val':  SERVO_DELTA_POS}

        # 二加一指
        elif self.mode == 'twone':
            self.cmds = [{'type': 'spd', 'val': 0} for _ in range(self.SERVO_NUM)] # 每次都注意清除之前的指令
            # 控二指
            if self.sub_mode == 'two':
                delta_pos = round(self.dis * self.TWO_RATIO)
                self.cmds[1] = {'type': 'pos', 'val': self.twone_pos[1] + delta_pos}
                self.cmds[2] = {'type': 'pos', 'val': self.twone_pos[2] + delta_pos}
            # 控一指
            else:
                delta_pos = round(self.dis * self.ONE_RATIO)
                self.cmds[0] = {'type': 'pos', 'val': self.twone_pos[0] + delta_pos}

            if self.tp_cmd == 11:
                self.cmds[3] = {'type': 'spd', 'val':  SERVO_DELTA_POS}
                self.cmds[4] = {'type': 'spd', 'val': -SERVO_DELTA_POS}
            elif self.tp_cmd == 12:
                self.cmds[3] = {'type': 'spd', 'val': -SERVO_DELTA_POS}
                self.cmds[4] = {'type': 'spd', 'val':  SERVO_DELTA_POS}
            
        # 停止
        elif self.mode == 'stop':
            self.cmds = [{'type': 'spd', 'val': 0} for _ in range(self.SERVO_NUM)]
        # 状态转换中
        elif self.mode == "trans" or self.mode == "initial":
            self.cmds = [{'type': 'pos', 'val': self.target_pos[i]} for i in range(self.SERVO_NUM)]

        self.set_all_servo()
        return

    test_mode = 0
    def servo_test(self):
        for i in range(self.SERVO_NUM):
            if self.servo_is_moving[i]:
                return

        target_pos = []
        for i in range(self.SERVO_NUM):
            target_pos.append([self.POS_LIMIT[i][0], 0.5*(self.POS_LIMIT[i][0]+self.POS_LIMIT[i][1])])

        print("mode: ", self.test_mode)
        print("target_pos: ", target_pos)
        for i in range(self.SERVO_NUM):
            write_pos = round(target_pos[i][self.test_mode])
            # print("servo_id: ", i, "write_pos: ", write_pos)
            self.write_servo(i, write_pos, 0, 1000)
            
        self.test_mode = not self.test_mode

def shutdown_hook():
    hand.portHandler.closePort()
    print("Program exit")

def signal_handler(sig, frame):
    rospy.signal_shutdown('Ctrl+C pressed')


if __name__ == "__main__":
    servo_port = rospy.get_param("servo_ports", default="/dev/ttyS5")
    baudrate = rospy.get_param("servo_baudrate", default=1000000)
    hand_id = rospy.get_param("hand_id", default=0)
    hand_id = 1
    script_dir = os.path.dirname(__file__)
    json_path = os.path.join(script_dir, 'servo_limit.json')
    with open(json_path, 'r', encoding='utf-8') as f:
        servo_limit_json = json.load(f)

    hand = HandCtrl(servo_port, baudrate, servo_limit_json[str(hand_id)])

    rospy.init_node("hand_ctrl")
    rospy.on_shutdown(shutdown_hook)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    hand_pub = rospy.Publisher("/control/hand/servo_read", Float32MultiArray, queue_size=10)
    hand_cmd_pub = rospy.Publisher("/control/hand/servo_cmd", Float32MultiArray, queue_size=10)
    servo_force_pub = rospy.Publisher("/control/hand/servo_force", Float32, queue_size=10)
    rospy.Subscriber("/control/hand/teach_pendant", Int32, hand.tpCmd_callback)
    rospy.Subscriber("/control/hand/distance", Float32, hand.dis_callback)
    rospy.Subscriber("/control/claw/ai_cmd", Float32MultiArray, hand.ai_callback)
    loop_rate = rospy.Rate(60)
    try:
        while not rospy.is_shutdown():
            hand.update()
            hand.servo_ctrl()
            msg = Float32MultiArray()
            msg.data = [hand.servo_pos[0], hand.servo_pos[1], hand.servo_pos[2], 
                        hand.servo_pos[3], hand.servo_pos[4]]
            hand_pub.publish(msg)
            msg = Float32MultiArray()
            msg.data = [hand.pos_cmd[0], hand.pos_cmd[1], hand.pos_cmd[2], 
                        hand.pos_cmd[3], hand.pos_cmd[4]]
            hand_cmd_pub.publish(msg)
            msg = Float32(hand.cal_servo_force())
            servo_force_pub.publish(msg)
            loop_rate.sleep()

    except rospy.ROSInterruptException:
        hand.portHandler.closePort()
