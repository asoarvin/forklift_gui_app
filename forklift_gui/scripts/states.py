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
    from cyngn_state_manager.srv import ForkliftConfirmations, ForkliftConfirmationsResponse
    from cyngn_state_manager.msg import ForkliftState, ForkliftPalletStackOfInterest
    from lev_msgs.msg import ForkControlPosition, ForkControlFeedback, ForkReport
    from lev_msgs.msg import ErrorReport

    import rospy
    from std_msgs.msg import String
    from geometry_msgs.msg import *
except Exception as e:
    print(e)
    print("STATE error importing ROS and/or Cyngn Libraries")
    pass

TOPIC_STATES = '/current_state' # "/cyngn_state_manager/state"
TOPIC_PALLET_OF_INTEREST = "/message_translation/pallet_stack_of_interest"
TOPIC_FORK_REPORT = "/cyngn/dbw/fork/status"
TOPIC_FORK_CTL_FEEDBACK = "/cyngn/dbw/fork/control_closed_loop"
TOPIC_ERROR_REPORT = '/drive/error'

GUI_PUBLISH_TOPIC = '/forklift_gui'

SERVICE_MODE = 'event_selection_srv'
SERVICE_PALLET = 'event_input_srv'
SERVICE_CONFIRM = 'event_confirmation_srv'
SERVICE_AUTONOMY = 'enable_autonomy_srv'

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
        self.current_state = STATE_SPACE[2]
        self.next_state = self.find_next_state()
        self.last_state_change = time.monotonic()
        self.new_state = False
        self.error_string = 'Error'
         # User input data
        self.user_enabled_autonomy = False
        self.user_selected_mode = 0                             # 1: pick, 2: place stack, 3: place ground
        self.user_selected_pallet = 0                           # 0 is bottom pallet, already valid, if not, is NONE
        self.user_inserted_forks = False
        self.confirmation_timeout = 60000                       # 1 minute hard coded timeout on confirmations
        # Gui / States interactions
        self.flag_ask_for_mode_selection = False                # For GUI Clock 
        self.flag_ask_mode_again = False                        # for close / re-open buttons
        self.flag_ask_for_pallet_selection = False              # For GUI Clock
        self.flag_ask_pallet_again = False                      # for close / re-open buttons
        self.flag_ask_for_confirm = False                       # confirmation service for placing pallets
        self.flag_mode_selected = False 
        self.flag_pallet_selected = False
        self.flag_confirmed = False
        # Traffic Light 
        self.show_green_light = False
        self.show_yellow_light = False
        self.show_red_light = False
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
        self.stack_height = 0
        self.pallet_stack_count = 0 # TODO
        self._pockets = []
        self._pocket_confidences = []
        # MISC / UNUSED
        self.fbw_enabled = False
        self.blocked = False
        self.fork_initialized = False
        self.pocket_detected = False
        self.fork_load_status = False
        self.fork_insertion_status = 0

        self.init_ros()
    
    def init_ros(self):
        try:
            # self.ros_thread = threading.Thread(target=lambda: ).start()
            rospy.init_node('forklift_gui_node', disable_signals=True) # 

            rospy.Subscriber(TOPIC_STATES, ForkliftState, self.states_callback)
            rospy.Subscriber(TOPIC_PALLET_OF_INTEREST, ForkliftPalletStackOfInterest, self.pallet_callback)
            rospy.Subscriber(TOPIC_FORK_REPORT, ForkReport, self.fork_control_pos_callback)
            rospy.Subscriber(TOPIC_FORK_CTL_FEEDBACK, ForkControlFeedback, self.fork_control_feedback_callback)
            rospy.Subscriber(TOPIC_ERROR_REPORT, ErrorReport, self.error_callback)
            # rospy.Subscriber(TOPIC_FORK_POS, ForkReport, self.fork_position_callback)
            # self.pub = rospy.Publisher(GUI_PUBLISH_TOPIC, String, queue_size=10)

            # schedule ros services servers for event selection and input
            rospy.Timer(rospy.Duration(0.25), self.pallet_selection_server, oneshot=False)
            rospy.Timer(rospy.Duration(0.25), self.event_selection_server, oneshot=False)
            rospy.Timer(rospy.Duration(0.25), self.confirmation_server, oneshot=False)
        except Exception as e:
            print(f"[ERROR] {e}\nERROR INITIALIZING ROS\n")
            pass

    def pallet_selection_server(self, dt):
        self.event_srv = rospy.Service(SERVICE_PALLET, ForkliftEventInput, self.send_pallet_selection)
        print("Ready to send pallet selection.")

    def event_selection_server(self, dt):
        self.pallet_srv = rospy.Service(SERVICE_MODE, ForkliftEventSelection, self.send_event_selection)
        print("Ready to send event selection.")

    def confirmation_server(self, dt):
        self.confirm_srv = rospy.Service(SERVICE_CONFIRM, ForkliftConfirmations, self.send_confirmation)
        print("Ready to send event confirmation.")

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

    def send_confirmation(self, req):
        print("confirmation service called")
        self.flag_ask_for_confirm = True
        st = time.monotonic()

        # wait for gui to flag confirmation button
        while not self.flag_confirmed:
            et = time.monotonic()
            dt = float(et - st)
            if dt >= self.confirmation_timeout:
                return bool(self.flag_confirmed)

        # we're returning a response, so set flag to false so it won't send again
        self.flag_confirmed = False

        return bool(self.flag_confirmed)

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
    
    def get_pocket_heights_str(self):
        ans = "Pocket Heights = ["
        count = 0
        try:
            for i in self._pockets:
                ans += f"{count}: {round(float(i), 2)}, "
                count += 1
        except:
            pass
            # print(f"ERROR getting pocket heights")
        ans += ']'
        return ans

    def get_pocket_confidence_str(self):
        ans = 'Pocket Confidence = ['
        try:
            for i in self._pocket_confidences:
                ans += f"{round(float(i), 2)}, "
        except:
            pass
            # print(f"ERROR getting pocket confidences")
        ans += ']'
        return ans

    def get_forks_position_str(self):
        return f"FORK (TILT, Y, Z): ({self.fork_tilt}, {self.fork_y}, {self.fork_z})"

    def get_forks_rel_position_str(self):
        return f"PALLET (X, Y, THETA): ({self.fork_rel_x}, {self.fork_rel_y}, {self.fork_rel_angle})"

    def get_fork_pocket_error_str(self):
        return f"ERROR (Y, Z): ({self.fork_pocket_error_y}, {self.fork_pocket_error_z})"

    def get_error_str(self):
        return f"[ERROR {self.error_code}]: {self.error_string}"

    def error(self, error_str):
        self.error_string = "ERROR: " + error_str

    def enable_autonomy(self, timeout=None):
        # rospy.wait_for_service('enable_autonomy', timeout=timeout)
        try:
            autonomous_mode = rospy.ServiceProxy(SERVICE_AUTONOMY, ForkliftEnableAutonomy)
            acknowledgement = autonomous_mode(True)

            if(acknowledgement.success):
                self.fbw_enabled = True
                print("Turning on autonomy")
            else: self.fbw_enabled = False

            return acknowledgement.success
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    def disable_autonomy(self, timeout=None):
        # rospy.wait_for_service('enable_autonomy', timeout=timeout)
        try:
            autonomous_mode = rospy.ServiceProxy(SERVICE_AUTONOMY, ForkliftEnableAutonomy)
            acknowledgement = autonomous_mode(False)

            if(acknowledgement.success):
                self.fbw_enabled = False
                print("Turning off autonomy")
            else: self.fbw_enabled = True

            return acknowledgement.success
        except rospy.ServiceException as e:
              print("Service call failed: %s"%e)

    def states_callback(self, msg):
        try:
            recv_state = str(msg.state)
            # new_info = msg.info

            for i in range(len(STATES_TOPIC_TRANSLATION)):
                if STATES_TOPIC_TRANSLATION[i] == recv_state:
                    print(f"{STATES_TOPIC_TRANSLATION[i] == recv_state} -- changing state to {STATE_SPACE[i]}")
                    self.change_state(STATE_SPACE[i])
                    return
        except:
            print("[GUI ROS] error parsing new state published")

    def pallet_callback(self, msg):
        try:
            self._pockets = msg.stack_info.pallet_pocket_height_m
            self._pocket_confidences = msg.stack_info.pallet_pocket_confidence
            
            self.fork_rel_x = round(float(msg.stack_info.stack_center_m.x), 2)
            self.fork_rel_y = round(float(msg.stack_info.stack_center_m.y), 2)
            self.fork_rel_angle = round(float(msg.stack_info.stack_center_m.z), 2)

            self.pallet_stack_count = int(len(self._pockets))
            
            if len(self._pockets) >= 1:
                self.pocket_detected = True
            else:
                self.pocket_detected = False
        except:
            print("[GUI ROS] error obtaining pallet stack count and other data")
        return

    def fork_control_pos_callback(self, msg):
        try:
            self.fork_y = round(float(msg.fork_y_position), 2)
            self.fork_z = round(float(msg.fork_z_position), 2)
            self.fork_tilt = round(float(msg.fork_tilt_position), 2)

            self.fork_insertion_status = int(msg.fork_insertion_status)
            if self.fork_insertion_status == 0: 
                self.show_green_light = True
                self.show_yellow_light = False
                self.show_red_light = False
            elif self.fork_insertion_status == 1:
                self.show_green_light = False
                self.show_yellow_light = True
                self.show_red_light = False
            elif self.fork_insertion_status == 2:
                self.show_green_light = False
                self.show_yellow_light = False
                self.show_red_light = True

            self.fork_load_status = bool(msg.fork_load_status)

        except:
            print("[GUI ROS] error obtaining fork position data")
        return

    def fork_control_feedback_callback(self, msg):
        try:
            self.fork_pocket_error_y = round(float(msg.pallet_pocket_y_error), 2)
            self.fork_pocket_error_z = round(float(msg.pallet_pocket_z_error), 2)
            self.distance_to_pallet = round(float(msg.distance_to_pallet), 2)
        except:
            print("[GUI ROS] error obtaining pocket error (y/z)")
        return
    
    def error_callback(self, msg):
        try:
            self.error_code = int(msg.error_code)
            self.error_string = str(msg.error_message)
        except:
            print("[GUI ROS] error obtaining Error Report")
        return