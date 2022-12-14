#!/usr/bin/env python3
import threading
import time
import yaml
import subprocess
import os

try:
    from cyngn_state_manager.srv import ForkliftEventInput, ForkliftEventInputResponse
    from cyngn_state_manager.srv import ForkliftEventSelection, ForkliftEventSelectionResponse
    from cyngn_state_manager.srv import ForkliftEnableAutonomy, ForkliftEnableAutonomyResponse
    from cyngn_state_manager.msg import ForkliftState, ForkliftPalletStackOfInterest
    from cyngn_msgs.msg import ForkControlPosition, ForkControlFeedback

    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import *
except:
    print("error importing ROS and/or Cyngn Libraries")
    pass

TOPIC_STATES = '/current_state'
# "/cyngn_state_manager/state"

TOPIC_PALLET_OF_INTEREST = "/message_translation/pallet_stack_of_interest"
TOPIC_FORK_CTL_POSITION = "/drive/forkposition"
TOPIC_FORK_CTL_FEEDBACK = "/drive/forkfeedback"

GUI_PUBLISH_TOPIC = '/forklift_gui'

STATE_SPACE = { 
    0: "STATE_ERROR",
    1: "STATE_MANUAL",
    2: "STATE_AUTONOMY_INIT",
    3: "STATE_PICK_INIT",
    4: "STATE_PICK_SELECT",
    5: "STATE_PICK_FORK_APPROX_ALIGN",
    6: "STATE_PICK_POCKET_DETECT_ENABLE",
    7: "STATE_PICK_CLOSEDLOOP_FORK_ENABLE",
    8: "STATE_PICK_POCKET_DETECT_CLOSEDLOOP_DISABLE",
    9: "STATE_PICK_LIFT_PALLET",
    10: "STATE_PLACEGND_INIT",
    11: "STATE_PLACEGND_PLACE_PALLET",
    12: "STATE_PLACESTACK_INIT",
    13: "STATE_PLACESTACK_FORK_APPROX_ALIGN",
    14: "STATE_PLACESTACK_PALLET_ALIGN",
    15: "STATE_PLACESTACK_PLACE_PALLET"
}

STATES_TOPIC_TRANSLATION = {
    0: "error",
    1: "manual",
    2: "autonomy_init",
    3: "pick_init",
    4: "pick_select",
    5: "fork_approx_align",
    6: "pocket_detect_enable",
    7: "closed_loop_fork_enable",
    8: "pocket_detect_closed_loop_disable",
    9: "list_pallet",
    10: "place_ground_init",
    11: "place_ground_place_pallet",
    12: "place_stack_init",
    13: "place_stack_fork_approx_align",
    14: "place_stack_pallet_align",
    15: "place_stack_place_pallet"
}

class ROS_STATES():
    def __init__(self):
        self.last_state = STATE_SPACE[0]
        self.current_state = STATE_SPACE[1]
        self.next_state = self.find_next_state()
        self.last_state_change = time.monotonic()
        self.new_state = False

        self.error_string = 'Error'

        # To be published
        self.user_enabled_autonomy = False
        
        self.flag_ask_for_mode_selection = False                # For GUI Clock 
        self.flag_ask_mode_again = False                        # for close / re-open buttons
        self.flag_ask_for_pallet_selection = False              # For GUI Clock
        self.flag_ask_pallet_again = False                      # for close / re-open buttons

        self.flag_mode_selected = False 
        self.flag_pallet_selected = False 

        self.user_selected_mode = 0             # 1: pick, 2: place stack, 3: place ground
        self.user_selected_pallet = 0           # 0 is bottom pallet, already valid, if not, is NONE
        self.user_inserted_forks = False

        # Fork Relative to Pocket Positions
        self.fork_rel_x = 0
        self.fork_rel_y = 0
        self.fork_rel_angle = 0

        # Fork Actual Position
        self.fork_tilt = 0
        self.fork_y = 0
        self.fork_z = 0

        # Fork/Pocket Error
        self.fork_pocket_error_y = 0
        self.fork_pocket_error_z = 0

        # Stack Info
        self.stack_height = 0
        self.pallet_stack_count = 4 # TODO

        # State Specific Info
        self.fbw_enabled = False
        self.blocked = False

        self.fork_initialized = False
        self.pocket_detected = False

        self.init_ros()
    
    def init_ros(self):
        try:
            # self.ros_thread = threading.Thread(target=lambda: ).start()
            rospy.init_node('forklift_gui_node', disable_signals=True) # 

            rospy.Subscriber(TOPIC_STATES, ForkliftState, self.states_callback)
            rospy.Subscriber(TOPIC_PALLET_OF_INTEREST, ForkliftPalletStackOfInterest, self.pallet_callback)
            # rospy.Subscriber(TOPIC_FORK_CTL_POSITION, ForkControlPosition, self.fork_control_pos_callback)
            # rospy.Subscriber(TOPIC_FORK_CTL_FEEDBACK, ForkControlFeedback, self.fork_control_feedback_callback)
            # self.pub = rospy.Publisher(GUI_PUBLISH_TOPIC, String, queue_size=10)

            # schedule ros services servers for event selection and input
            rospy.Timer(rospy.Duration(0.25), self.pallet_selection_server, oneshot=False)
            rospy.Timer(rospy.Duration(0.25), self.event_selection_server, oneshot=False)
        except Exception as e:
            print(f"[ERROR] {e}\nERROR INITIALIZING ROS\n")
            pass

    def pallet_selection_server(self, dt):
        self.event_srv = rospy.Service('event_input', ForkliftEventInput, self.send_pallet_selection)
        print("Ready to send pallet selection.")

    def event_selection_server(self, dt):
        self.pallet_srv = rospy.Service('event_selection', ForkliftEventSelection, self.send_event_selection)
        print("Ready to send event selection.")

    def send_pallet_selection(self, req):
        print("pallet selection called")
        self.flag_ask_for_pallet_selection = True

        # wait for gui to flag states
        while not self.flag_pallet_selected:
            time.sleep(0.1)
            # print(self.flag_pallet_selected)
            # tmp = 1+2
        
        # we're returning a response, so set flag to false so it won't send again
        self.flag_pallet_selected = False

        return int(self.user_selected_pallet)

    def send_event_selection(self, req):
        print("mode selection called")
        self.flag_ask_for_mode_selection = True

        # wait for gui to flag states
        while not self.flag_mode_selected:
            time.sleep(0.1)
            # print(self.flag_mode_selected)

        # we're returning a response, so set flag to false so it won't send again
        self.flag_mode_selected = False

        return int(self.user_selected_mode)

    def find_next_state(self):
        tmp = ''
        if self.current_state == STATE_SPACE[0]: tmp = STATE_SPACE[1]
        elif self.current_state == STATE_SPACE[1]: tmp = STATE_SPACE[2]
        elif self.current_state == STATE_SPACE[2]: tmp = STATE_SPACE[3] + ", " + STATE_SPACE[10] + ", " + STATE_SPACE[12]
        elif self.current_state == STATE_SPACE[3]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[4]
        elif self.current_state == STATE_SPACE[4]: tmp = STATE_SPACE[4] + ", " +  STATE_SPACE[5]
        elif self.current_state == STATE_SPACE[5]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[6]
        elif self.current_state == STATE_SPACE[6]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[7]
        elif self.current_state == STATE_SPACE[7]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[8]
        elif self.current_state == STATE_SPACE[8]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[9]
        elif self.current_state == STATE_SPACE[9]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[1]
        elif self.current_state == STATE_SPACE[10]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[11]
        elif self.current_state == STATE_SPACE[11]: tmp = STATE_SPACE[0] + ", " +  STATE_SPACE[13]
        elif self.current_state == STATE_SPACE[12]: tmp = STATE_SPACE[4] + ", " +  STATE_SPACE[5]
        elif self.current_state == STATE_SPACE[13]: tmp = STATE_SPACE[4] + ", " +  STATE_SPACE[5]
        elif self.current_state == STATE_SPACE[14]: tmp = STATE_SPACE[4] + ", " +  STATE_SPACE[5]
        elif self.current_state == STATE_SPACE[15]: tmp = STATE_SPACE[4] + ", " +  STATE_SPACE[5]
        return tmp
    
    def change_state(self, next_state):
        self.new_state = True
        self.last_state = self.current_state
        self.current_state = next_state
        self.next_state = self.find_next_state()
        self.last_state_change = time.monotonic()

    def get_pallet_stack_count(self):
        '''returns the amount of pallets on the detect stack from the ROS TOPIC'''
        return self.pallet_stack_count
    
    def get_forks_position_str(self):
        return f"FORK (TILT, Y, Z): ({self.fork_tilt}, {self.fork_y}, {self.fork_z})"

    def get_forks_rel_position_str(self):
        return f"FORK REL. (X, Y, THETA): ({self.fork_rel_x}, {self.fork_rel_y}, {self.fork_rel_angle})"

    def get_fork_pocket_error_str(self):
        return f"FORK/POCKET ERROR (Y, Z): ({self.fork_pocket_error_y}, {self.fork_pocket_error_z})"

    def error(self, error_str):
        self.error_string = "ERROR: " + error_str

    def enable_autonomy(self, timeout=None):
        # rospy.wait_for_service('enable_autonomy', timeout=timeout)
        try:
            autonomous_mode = rospy.ServiceProxy('enable_autonomy', ForkliftEnableAutonomy)
            acknowledgement = autonomous_mode(True)
            if(acknowledgement.success):
              print("Turning on autonomy")

            return acknowledgement.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def disable_autonomy(self, timeout=None):
        # rospy.wait_for_service('enable_autonomy', timeout=timeout)
        try:
            autonomous_mode = rospy.ServiceProxy('enable_autonomy', ForkliftEnableAutonomy)
            acknowledgement = autonomous_mode(False)
            if(acknowledgement.success):
              print("Turning off autonomy")

              return acknowledgement.success
        except rospy.ServiceException as e:
              print("Service call failed: %s"%e)

    def states_callback(self, msg):
        try:
            recv_state = str(msg.state)
            # print(f"new state published was {recv_state}")

            for i in range(len(STATES_TOPIC_TRANSLATION)):
                if STATES_TOPIC_TRANSLATION[i] == recv_state:
                    print(f"{STATES_TOPIC_TRANSLATION[i] == recv_state} -- changing state to {STATE_SPACE[i]}")
                    self.change_state(STATE_SPACE[i])
                    return
        except:
            print("[GUI ROS] error parsing new state published")

    ''' FORKLIFTPALLETSTACKOFINTEREST
    uint64 id
    uint64 lifecycle

    geometry_msgs/Point32 stack_center_m
    geometry_msgs/Vector3 velocity_mps
    geometry_msgs/Point32 dimension_m
    float32 range_xy_m
    float32 orientation_rad

    float32[] pallet_pocket_height_m
    float32[] pallet_pocket_confidence
    '''
    def pallet_callback(self, msg):
        try:
            arr_pockets = msg.stack_info.pallet_pocket_height_m
            arr_pocket_confidence = msg.stack_info.pallet_pocket_confidence
            
            self.fork_rel_x = float(msg.stack_info.stack_center_m.x)
            self.fork_rel_y = float(msg.stack_info.stack_center_m.y)
            self.fork_rel_angle = float(msg.stack_info.stack_center_m.z)

            self.pallet_stack_count = int(len(arr_pockets))
            
            if len(arr_pockets) >= 1:
                self.pocket_detected = True
            else:
                self.pocket_detected = False
        except:
            print("[GUI ROS] error obtaining pallet stack count and other data")
        return

    ''' FORKCONTROLPOSITION.MSG
            # [-1 .. 1]
            float32 fork_y_position

            # [0 .. 1]
            float32 fork_z_position

            # [-1 .. 1]
            float32 fork_tilt_position
    '''
    def fork_control_pos_callback(self, msg):
        try:
            self.fork_y = msg.fork_y_position
            self.fork_z = msg.fork_z_position
            self.fork_tilt = msg.fork_tilt_position
        except:
            print("[GUI ROS] error obtaining fork position data")
        return

    '''FORKCONTROLFEEDBACK.MSG
    # [-1 .. 1]
    float32 pallet_pocket_y_error

    # [-1 .. 1]
    float32 pallet_pocket_z_error

    # [0 .. 1]
    float32 distance_to_pallet
    '''
    def fork_control_feedback_callback(self, msg):
        try:
            self.fork_pocket_error_y = msg.pallet_pocket_y_error
            self.fork_pocket_error_z = msg.pallet_pocket_z_error
        except:
            print("[GUI ROS] error obtaining pocket error (y/z)")
        return