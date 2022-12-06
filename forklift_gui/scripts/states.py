#!/usr/bin/env python
import threading
import time
import yaml

try:
    import rospy
    from std_msgs.msg import String
except Exception as e:
    print(e)
    pass

# TODO: make this the state manager topic
FILE_TOPICS_YAML = 'topics.yaml'
TOPIC_STATES = '/vehicle/odometry'
TOPIC_OUSTER = '/vehicle/odometry'
TOPIC_VISIONARY_T = '/vehicle/odometry'
TOPIC_STW = '/vehicle/odometry'
TOPIC_TIM = '/vehicle/odometry'

GUI_PUBLISH_TOPIC = '/forklift_gui'

'''
We recieve the current state from the ros topic mentioned above. 

from this we need to direct the screen manager in forklift_gui.py

'''
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
        self.user_selected_mode = 0
        self.user_selected_pallet = 0
        self.user_selected_pallet_valid = False
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

        self.pick_CL_error = False
        self.pick_CL_min_range_reached = False

        self.forks_inserted = False
        self.forks_lifted = False

        self.placement_zone_clear = False
        self.placement_complete = False

        self.stack_height_alignment = False
        self.pallet_aligned = False
        
        #self.load_topics_yaml()

        # TODO: use yaml topics and subscribe with a callback for each
        try:
            self.ros_thread = threading.Thread(target=lambda: rospy.init_node('cyngn_fork_gui_node', disable_signals=True)).start()
            rospy.Subscriber(TOPIC_STATES, String, self.states_callback)
            self.pub = rospy.Publisher(GUI_PUBLISH_TOPIC, String, queue_size=10)
        except Exception as e:
            print(f"[ERROR] {e}\nDID NOT CREATE ROS\n")
            pass
    
    def load_topics_yaml(self):
        try:
            with open(FILE_TOPICS_YAML, 'r') as file:
                    self.topics = yaml.safe_load(file)
            
            self.ouster_topic = self.topics['ouster']
            self.vis_t__topic = self.topics['visionary_t']
            self.tim_topic = self.topics['tim']
            self.stw_topic = self.topics['stw']
        except:
            print(f"COULDN'T LOAD TOPICS YAML {FILE_TOPICS_YAML}")

    def states_callback(self, msg): # TODO
        print(msg.data)
        try:
            if msg != self.current_state:
                self.change_state(msg)
        except Exception as e:
            print(f"ERROR: {e}\n")
        return

    def ouster_view_obstructed_callback(self, msg):
        try:
            print(f"recieved {msg.data}")
            self.blocked = bool(msg.data)
        except Exception as e:
            print(f"ERROR {e}\n")

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
        self.change_state(STATE_SPACE[0])
