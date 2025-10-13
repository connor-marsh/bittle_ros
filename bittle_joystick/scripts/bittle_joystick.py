#!/usr/bin/env python3
#Author: Aman Jiwani 
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64
import numpy as np

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

    def joy_callback(self, joy_msg):
        # Example mapping: Joystick axes control joint positions
        joint1_command = joy_msg.axes[0] 

        # Publish control commands
        self.joint1_pub.publish(joint1_command)

if __name__ == '__main__':
    try:
        controller = RobotArmController()
        joint_names = ['shrrt', 'shrrs', 'shrft', 'shrfs', 'shlrt', 'shlrs', 'shlft', 'shlfs', 'neck'] 
        x = 0.0
        while True:
            for joint_pub in controller.joint_pubs:
                joint_pub.publish(np.sin(3.14*x))
            x += 0.005
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

