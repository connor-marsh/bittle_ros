#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState, DeleteModel, SpawnModel, SetModelConfiguration
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
from controller_manager_msgs.srv import LoadController, SwitchController, SwitchControllerRequest
from std_srvs.srv import Empty # For Pause/Unpause Physics
import time

def reset_robot_sequence(robot_name, urdf_xml, initial_pose, joint_names, initial_joint_positions, controller_list):
    rospy.init_node('robot_reset_script')

    # --- 1. Get Initial State (Not strictly necessary for a clean reset, but good practice)
    # We'll skip getting the *current* state and focus on the clean respawn, 
    # but the service for getting the state is GetModelState.
    rospy.loginfo("Starting Robot Reset Sequence...")

    # --- 2. Delete the Robot
    rospy.loginfo("2. Deleting old model...")
    rospy.wait_for_service('/gazebo/delete_model')
    delete_model_proxy = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
    try:
        delete_model_proxy(robot_name)
        rospy.loginfo("Model deleted.")
    except rospy.ServiceException as e:
        rospy.logwarn("Delete model failed (maybe it didn't exist?): %s", e)

    # --- 3. Pause Physics
    rospy.loginfo("3. Pausing physics...")
    rospy.wait_for_service('/gazebo/pause_physics')
    pause_physics_proxy = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    pause_physics_proxy()

    # --- 4. Spawn the Robot Again
    rospy.loginfo("4. Spawning new model...")
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_model_proxy = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    try:
        spawn_model_proxy(
            model_name=robot_name,
            model_xml=urdf_xml,
            robot_namespace=ROBOT_NAMESPACE,
            initial_pose=initial_pose,
            reference_frame='world'
        )
        rospy.loginfo("Model spawned successfully.")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn model failed: %s", e)
        return False
    
    # Wait briefly for Gazebo to process the spawn
    time.sleep(0.5)

    # --- 5. Set Initial Joint States (Set Model Configuration)
    # This is critical to set the joint positions before controllers are loaded.
    rospy.loginfo("5. Setting initial joint configuration...")
    rospy.wait_for_service('/gazebo/set_model_configuration')
    set_model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
    try:
        set_model_config_proxy(
            model_name=robot_name,
            urdf_param_name='robot_description', # Assumes URDF is on param server
            joint_names=joint_names,
            joint_positions=initial_joint_positions
        )
        rospy.loginfo("Joint configuration set.")
    except rospy.ServiceException as e:
        rospy.logerr("Set model configuration failed: %s", e)
        return False

    # --- 6. Load all controllers
    rospy.loginfo("6. Loading controllers...")
    rospy.wait_for_service(f'{ROBOT_NAMESPACE}/controller_manager/load_controller')
    load_controller_proxy = rospy.ServiceProxy(f'{ROBOT_NAMESPACE}/controller_manager/load_controller', LoadController)
    
    # Load all controllers sequentially
    for controller in controller_list:
        try:
            load_controller_proxy(name=controller)
            rospy.loginfo(f"Controller {controller} loaded.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to load controller {controller}: %s", e)
            return False

    # --- 7. Start all controllers (Switch Controller)
    # This is where we send the request and it waits for unpause (Step 8)
    rospy.loginfo("7. Starting controllers (waiting for unpause)...")

    print("WAITINF FOR SWITCH CONTROLLER")
    rospy.wait_for_service(f'{ROBOT_NAMESPACE}/controller_manager/switch_controller')
    print("FINISHED WAITING")
    switch_controller_proxy = rospy.ServiceProxy(f'{ROBOT_NAMESPACE}/controller_manager/switch_controller', SwitchController)
    
    switch_request = SwitchControllerRequest()
    switch_request.start_controllers = controller_list
    switch_request.stop_controllers = [] # None to stop
    switch_request.strictness = SwitchControllerRequest.STRICT # Use 2 for STRICT
    switch_request.timeout = 0.0

    # --- 8. Unpause Physics
    # The moment physics is unpaused, the switch_controller call is usually resolved.
    rospy.loginfo("8. Unpausing physics...")
    rospy.wait_for_service('/gazebo/unpause_physics')
    unpause_physics_proxy = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    unpause_physics_proxy()

    print("WAITINF FOR SWITCH CONTROLLER BUT CALL THIS TIME")
    switch_future = switch_controller_proxy.call(switch_request)
    print("FINISHED WAITING")
    
    

    # Check the result of the switch (wait for future if needed, but unpause should trigger it)
    if switch_future.ok:
        rospy.loginfo("✅ Robot reset successful! Controllers active and /joint_states should be publishing.")
        return True
    else:
        rospy.logerr("❌ Controller switch failed after unpausing physics.")
        return False

if __name__ == '__main__':
    # --- CONFIGURATION ---
    ROBOT_NAME = 'bittle'
    ROBOT_NAMESPACE = '/' # Ensure this matches your URDF's <robotNamespace>
    
    # 1. Get the URDF from the parameter server
    try:
        URDF_XML = rospy.get_param('robot_description')
    except KeyError:
        rospy.logerr("FATAL: 'robot_description' parameter not found on the parameter server.")
        exit(1)
        
    # 2. Define the starting pose (position and orientation)
    INITIAL_POSE = Pose()
    INITIAL_POSE.position.x = 0.0
    INITIAL_POSE.position.y = 0.0
    INITIAL_POSE.position.z = 0.8
    # Orientation can be left at default (0,0,0,1) for no rotation

    joint_names = ["left_back_shoulder",
                    "left_back_knee",
                    "left_front_shoulder",
                    "left_front_knee",
                    "right_back_shoulder",
                    "right_back_knee",
                    "right_front_shoulder",
                    "right_front_knee"]

    # 3. Define the initial joint positions
    JOINT_NAMES = [name+"_joint" for name in joint_names] # List ALL controlled joints
    # INITIAL_JOINT_POSITIONS = [0.5 for name in joint_names] # Corresponding positions (radians/meters)
    INITIAL_JOINT_POSITIONS = [0.6, 0, 0.6, 0, -0.6, 0, -0.6, 0] # Corresponding positions (radians/meters)

    # 4. Define the controllers to be loaded/started
    CONTROLLER_LIST = ['joint_state_controller'] + [name + "_joint_position_controller" for name in joint_names]

    # --- RUN THE SEQUENCE ---
    reset_robot_sequence(ROBOT_NAME, URDF_XML, INITIAL_POSE, JOINT_NAMES, INITIAL_JOINT_POSITIONS, CONTROLLER_LIST)