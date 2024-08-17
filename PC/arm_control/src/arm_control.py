#!/usr/bin/python3

import rospy
import time
import numpy as np
import airbot
import math
from std_msgs.msg import Int32, Bool, Float32
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Pose

class ArmsCtrl:
    def __init__(self, can0 = "can0", can1 = "can1"):
        self.arms = {
            'teacher':  airbot.create_agent(end_mode="none", can_interface=can0, vel=3),
            'follower': airbot.create_agent(end_mode="none", can_interface=can1, vel=3)
        }
        self.recvd_cnt = {
            'teacher':  0,
            'follower': 0
        }
        self.joint_pos = {
            'teacher':  [0]*6,
            'follower': [0]*6
        }
        self.jointState = {
            'teacher':  JointState(),
            'follower': JointState()
        }
        self.pub_msg = {
            'teacher':  Float32MultiArray(),
            'follower': Float32MultiArray()
        }

        self.pos = [0]*6
        self.vel = [0]*6
        self.kp  = [0]*6
        self.kd  = [0]*6
        self.tor = [0]*6

        self.arms['teacher'].manual_mode()

    def read_teacher(self):
        self.joint_pos['teacher'] = self.arms['teacher'].get_current_joint_q()
        self.save_joint_state('teacher')
        time.sleep(0.001)
        success = self.arms['teacher'].set_target_joint_mit(self.pos, self.vel, self.kp, self.kd, self.tor)
        if success: 
            self.recvd_cnt['teacher'] += 1

    def write_follower(self):
        self.joint_pos['follower'] = self.arms['follower'].get_current_joint_q()
        self.save_joint_state('follower')
        time.sleep(0.001)
        success = self.arms['follower'].set_target_joint_q(self.joint_pos['teacher'], blocking=False, use_planning=False, vel=5)
        if success: 
            self.recvd_cnt['follower'] += 1
    
    def save_joint_state(self, arm_name):
        self.jointState[arm_name].name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        self.jointState[arm_name].position = self.joint_pos[arm_name]
        self.jointState[arm_name].velocity = [0]*6
        self.jointState[arm_name].effort = [0]*6
        self.jointState[arm_name].header.stamp = rospy.Time.now()
        self.jointState[arm_name].header.frame_id = arm_name + "/joint_states"
        self.pub_msg[arm_name].data = self.joint_pos[arm_name]

    def ctrl_loop(self):
        self.read_teacher()
        time.sleep(0.001)
        self.write_follower()


if __name__ == "__main__":
    arms_ctrl = ArmsCtrl("can1", "can0")
    
    rospy.init_node("arms_node")
    teacher_pub = rospy.Publisher("/control/arms/teacher_joint", Float32MultiArray, queue_size=10)
    follower_pub = rospy.Publisher("/control/arms/follower_joint", Float32MultiArray, queue_size=10)

    loop_rate = rospy.Rate(120)
    loop_cnt = 0
    try:  
        while not rospy.is_shutdown():
            arms_ctrl.ctrl_loop()

            if not (loop_cnt%2):
                teacher_pub.publish(arms_ctrl.pub_msg['teacher'])
                follower_pub.publish(arms_ctrl.pub_msg['follower'])

            # if not (loop_cnt%100):
            #     print("success rate:", (arms_ctrl.recvd_cnt['teacher']*100/loop_cnt), "%, ",  (arms_ctrl.recvd_cnt['follower']*100/loop_cnt), "%")
            #     print("tea:", arms_ctrl.arms['teacher'].get_current_joint_q())
            #     print("fol:", arms_ctrl.arms['follower'].get_current_joint_q())
            loop_cnt += 1
            loop_rate.sleep()
    except KeyboardInterrupt:
        
        pass

