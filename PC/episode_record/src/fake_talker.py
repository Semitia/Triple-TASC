#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray
import numpy as np

def talker():
    pub1 = rospy.Publisher('/control/hand/servo_read', Float32MultiArray, queue_size=10)
    pub2 = rospy.Publisher("/control/hand/motor_read", Float32MultiArray, queue_size=10)
    pub3 = rospy.Publisher("/control/hand/sensor", Float32MultiArray, queue_size=10)
    pub4 = rospy.Publisher('/control/hand/servo_cmd', Float32MultiArray, queue_size=10)
    pub5 = rospy.Publisher("/control/hand/motor_cmd", Float32MultiArray, queue_size=10)
    rospy.init_node('fake_talker', anonymous=True)
    rate = rospy.Rate(60)  # 60Hz

    while not rospy.is_shutdown():

        rospy.loginfo("start talking")
        data1 = np.array([1, 1, 1, 1, 1])
        msg1 = Float32MultiArray(data=data1)
        pub1.publish(msg1)

        data2 = np.array([2,2,2])
        msg2 = Float32MultiArray(data=data2)
        pub2.publish(msg2)

        data3 = np.array([3,4,3,4,3,4])
        msg3 = Float32MultiArray(data=data3)
        pub3.publish(msg3)

        rospy.loginfo("start talking")
        data4 = np.array([1.5, 1.5, 1.5, 1.5, 1.5])
        msg4 = Float32MultiArray(data=data4)
        pub4.publish(msg4)

        data5 = np.array([2.5,2.5,2.5])
        msg5 = Float32MultiArray(data=data5)
        pub5.publish(msg5)

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
