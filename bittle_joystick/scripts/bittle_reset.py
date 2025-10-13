#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import numpy as np
import time

class RobotArmController:
    def __init__(self):
        rospy.init_node('robot_controller')

        # Define publishers for joint control commands

        joint_names = ['shrrt', 'shrrs', 'shrft', 'shrfs', 'shlrt', 'shlrs', 'shlft', 'shlfs', 'neck'] 
        self.joint_pubs = []
        for joint_name in joint_names:
            self.joint_pubs.append(rospy.Publisher('/' + joint_name + '_joint_position_controller/command', Float64, queue_size=10))
        # Subscribe to the joystick topic
        #rospy.Subscriber('joy', Joy, self.joy_callback)

walk_shoulders = [
    [0.76272625, 1.13457267, 0.76272625, 1.13457267],
    [0.93000985, 1.18074076, 0.93000985, 1.18074076],
    [1.05577914, 0.99622112, 1.05577914, 0.99622112],

    [1.13457267, 0.76272625, 1.13457267, 0.76272625],
    [1.18074076, 0.93000985, 1.18074076, 0.93000985],
    [0.99622112, 1.05577914, 0.99622112, 1.05577914]
]

walk_knees = [
    [0.33109918, 0.05484791, 0.33109918, 0.05484791],
    [0.28922337, 0.39589408, 0.28922337, 0.39589408],
    [0.19638032, 0.42164591, 0.19638032, 0.42164591],

    [0.05484791, 0.33109918, 0.05484791, 0.33109918],
    [0.39589408, 0.28922337, 0.39589408, 0.28922337],
    [0.42164591, 0.19638032, 0.42164591, 0.19638032]
]
walk_knees = [[0, 0, 0, 0]]
walk_shoulders = [[0, 0, 0, 0]]


if __name__ == '__main__':
    try:
        controller = RobotArmController()
        joint_names = ['shrrt', 'shrrs', 'shrft', 'shrfs', 'shlrt', 'shlrs', 'shlft', 'shlfs', 'neck'] 
        x = 0
        while True:
            y = 0
            for joint_pub in controller.joint_pubs[0:-1]:
                useShoulder = y % 2 == 1
                angle = (walk_shoulders if useShoulder else walk_knees)[x][int(y/2.0)]
                if angle is not None:
                    joint_pub.publish(angle)
                y+=1
            x += 1
            x %= len(walk_shoulders)
            time.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



