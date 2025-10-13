#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import numpy as np
import time
from copy import deepcopy

class RobotArmController:
    def __init__(self):
        rospy.init_node('robot_controller')

        # Define publishers for joint control commands
        joint_names = ['neck', 'shlfs', 'shlft', 'shlrs', 'shlrt', 'shrfs', 'shrft', 'shrrs', 'shrrt']
        self.joint_pubs = []
        for joint_name in joint_names:
            self.joint_pubs.append(rospy.Publisher('/' + joint_name + '_joint_position_controller/command', Float64, queue_size=10))
        self.neck_pub = self.joint_pubs[0]
        self.leg_pubs = self.joint_pubs[1:]
        self.shoulder_pubs = self.joint_pubs[1::2]
        self.toe_pubs = self.joint_pubs[2::2]

        self.joint_states = [0. for i in range(9)]
        self.leg_states = self.joint_states[1:]
        self.shoulder_states = self.joint_states[1::2]
        self.toe_states = self.joint_states[2::2]

        # Subscribe to the current joint state topic
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

    def joint_states_callback(self, js):
        self.joint_states = js.position
        self.leg_states = self.joint_states[1:]
        self.shoulder_states = self.joint_states[1::2]
        self.toe_states = self.joint_states[2::2]
        
    ### BLOCKING FUNCTION
    ### TODO make this work on sim time not python timer time
    def go_to_angles(self, angles, type="leg", max_diff_per_timestep=0.05, timestep=0.02, deadband=0.02):
        print("Starting go_to_angles")
        if type == "leg":
            pubs = self.leg_pubs
            states = self.leg_states
        elif type == "shoulder":
            pubs = self.shoulder_pubs
            states = self.shoulder_states
        elif type == "toe":
            pubs = self.toe_pubs
            states = self.toe_states
        else:
            pubs = self.joint_pubs
            states = self.joint_states
        
        temp_states = list(deepcopy(states))
        while not all(angle == None for angle in temp_states):
            #print(f"States: {temp_states}  |  Goals: {angles}")
            for i in range(len(temp_states)):
                if temp_states[i] == None:
                    continue
                dir, mag = get_angle_dir(temp_states[i], angles[i])
                if mag < deadband:
                    temp_states[i] = None
                    continue
                temp_states[i] += min(max_diff_per_timestep, mag)*dir
                pubs[i].publish(temp_states[i])
            time.sleep(timestep)
        print("Finished go_to_angles")

def get_angle_dir(start_angle, target_angle):
    start_angle = start_angle % (2*np.pi)
    target_angle = target_angle % (2*np.pi)
    direct_diff = target_angle - start_angle
    if direct_diff > np.pi:
        shortest_diff = direct_diff - 2*np.pi
    elif direct_diff < -np.pi:
        shortest_diff = direct_diff + 2*np.pi
    else:
        shortest_diff = direct_diff
    if shortest_diff == 0:
        direction = 0
    elif shortest_diff > 0:
        direction = 1
    else:
        direction = -1
    magnitude = abs(shortest_diff)
    if magnitude == np.pi:
        direction = 1
    return direction, magnitude


walk_shoulders = [
    [0.76272625, 1.13457267, 0.76272625, 1.13457267],
    [0.93000985, 1.18074076, 0.93000985, 1.18074076],
    [1.05577914, 0.99622112, 1.05577914, 0.99622112],

    [1.13457267, 0.76272625, 1.13457267, 0.76272625],
    [1.18074076, 0.93000985, 1.18074076, 0.93000985],
    [0.99622112, 1.05577914, 0.99622112, 1.05577914]
]

walk_toes = [
    [0.33109918, 0.05484791, 0.33109918, 0.05484791],
    [0.28922337, 0.39589408, 0.28922337, 0.39589408],
    [0.19638032, 0.42164591, 0.19638032, 0.42164591],

    [0.05484791, 0.33109918, 0.05484791, 0.33109918],
    [0.39589408, 0.28922337, 0.39589408, 0.28922337],
    [0.42164591, 0.19638032, 0.42164591, 0.19638032]
]
gait=[]
for i in range(6):
    gait.append([])
    for j in range(4):
        gait[i].append(walk_shoulders[i][j])
        gait[i].append(walk_toes[i][j])
gait = np.array(gait)
gait[:, [0, 2]] = gait[:, [2, 0]]
gait[:, [1, 3]] = gait[:, [3, 1]]
gait[:, [0, 2, 4, 6]] -= np.pi/4
gait[:, [1, 3, 5, 7]] += np.pi/4

#gait = np.array([[0 for i in range(8)], [np.pi/2 for i in range(8)]])
#gait[:, [0, 2, 4, 6]] /= 3
#gait[:, [1, 3, 5, 7]] /= 1
# account for front shoulders have angle directions flipped
gait[:, 0] *= -1
gait[:, 4] *= -1
gait = np.flip(gait, axis=0)

if __name__ == '__main__':
    try:
        controller = RobotArmController()
        x = 0
        while not rospy.is_shutdown():
            angles = gait[x]
            controller.go_to_angles(angles)
            x += 1
            x %= len(gait)
            time.sleep(0.2)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



