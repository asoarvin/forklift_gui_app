#!/usr/bin/env python3

from __future__ import print_function
import os
import subprocess
import threading
# from cefpython3 import cefpython as cef
import platform
import sys
print (sys.version)

from kivy.app import App
# from kivy.garden.cefpython import CefBrowser, cefpython
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
import kivy.core.text
from kivy.app import App
from kivy.base import EventLoop
from kivy.uix.image import Image
from kivy.uix.image import AsyncImage
from kivy.clock import Clock
from kivy.graphics.texture import Texture
from kivy.uix.boxlayout import BoxLayout
from kivy.core.window import Window
from kivy.properties import ObjectProperty
from kivy.graphics import Color
from kivy.uix.checkbox import CheckBox
from kivy.uix.progressbar import ProgressBar
from kivy.uix.screenmanager import ScreenManager, Screen, FadeTransition, SwapTransition, RiseInTransition, FallOutTransition 
from kivy.uix.recycleview import RecycleView
from kivy.uix.recycleview.views import RecycleDataViewBehavior
from kivy.uix.label import Label
from kivy.properties import BooleanProperty
from kivy.uix.recycleboxlayout import RecycleBoxLayout
from kivy.uix.behaviors import FocusBehavior
from kivy.uix.recycleview.layout import LayoutSelectionBehavior
from kivy.uix.scrollview import ScrollView
from kivy.properties import StringProperty
from kivy.lang import Builder
from kivy.properties import ListProperty, StringProperty, ObjectProperty
from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.uix.popup import Popup

import rospkg
import rospy
import states
from states import *
from cyngn_state_manager.srv import ForkliftEventInput, ForkliftEventInputResponse
from cyngn_state_manager.srv import ForkliftEventSelection
from cyngn_state_manager.srv import ForkliftEnableAutonomy, ForkliftEnableAutonomyResponse

class ScreenManagement(ScreenManager):
    def __init__(self, **kwargs):
        super(ScreenManagement, self).__init__(**kwargs)

''' LANDING PAGE WINDOW THAT WILL SHOW FIRST '''
class MainWindow(Screen):
    def __init__(self, **kwargs):
        super(MainWindow, self).__init__(**kwargs)

        self.cyan = (0.0, 1.0, 1.0, 1.0) #CYN-YAN
        self.state_label_colors = (0.0, 1.0, 1.0, 1.0)
        self.white = (1.0, 1.0, 1.0, 1.0)
        self.red = (1.0, 0.0, 0.0, 1.0)
        self.orange = (1.0, (151/255.0), 0.0, 1.0) #Orange

        self.STATE_MACHINE = states.ROS_STATES()
        # print(dir(self.STATE_MACHINE))
        self.popup_width = 800
        self.popup_height = 700
        self.popup_frame_width = 800
        self.popup_frame_height = 700

        Clock.schedule_interval(self.check_for_mode_popup, 0.25)
        Clock.schedule_interval(self.check_for_pallet_popup, 0.25)
        Clock.schedule_interval(self.check_for_new_state, 0.25)

        # Clock.schedule_interval(self.STATE_MACHINE.event_selection_server, 0.25)
        # Clock.schedule_interval(self.STATE_MACHINE.pallet_selection_server, 0.25)

        self.load_screen()

    def load_screen(self):

        # HEADER ELEMENTS
        ##########################

        self.rospack = rospkg.RosPack()
        package_path = self.rospack.get_path('forklift_gui')
        
        cyngn_logo_path = os.path.join(package_path, "cyngn_logo.png")
        back_icon_path = os.path.join(package_path,"back.png")
        close_icon_path = os.path.join(package_path,"close.png")
        settings_icon_path = os.path.join(package_path,"settings.png")
        
        self.header = BoxLayout(orientation ='horizontal', size_hint_y = None, height = 150)
        self.cyngn_logo = Image(source = cyngn_logo_path, size_hint_y = 0.8, allow_stretch = True)
        self.label = Label(text=str("Cyngn Forklift GUI"), font_size = '45dp', size_hint = (1, 0.8))
        self.settings_button = Button(height = 100, width = 100,  size_hint = (None, None), background_normal = settings_icon_path )
        self.settings_button.bind(on_press = self.settings_button_callback)
        self.close_button = Button( height = 100, width = 100,  size_hint = (None, None), background_normal = close_icon_path )
        self.close_button.bind(on_press = self.close_button_callback)

        self.header.add_widget(self.cyngn_logo)
        self.header.add_widget(self.label)
        # self.header.add_widget(self.settings_button)
        self.header.add_widget(self.close_button)
        
        # MAN / AUTO MODE
        ########################## 

        self.autonomy_buttons_header = BoxLayout(orientation ='horizontal', padding = 10, size_hint_y = None, height = 150)
        self.enable_autonomy_button = Button(text = "Autonomy", font_size = 50)
        self.enable_autonomy_button.bind(on_press = self.enable_autonomy_callback)
        self.disable_autonomy_button = Button(text = "Manual", font_size = 50)
        self.disable_autonomy_button.bind(on_press = self.disable_autonomy_callback)

        self.autonomy_buttons_header.add_widget(self.enable_autonomy_button)
        self.autonomy_buttons_header.add_widget(self.disable_autonomy_button)

        # MAIN CONTENT
        ##########################
        self.content = BoxLayout(orientation ='horizontal', padding=10)
        self.load_content()

        # CREATING THE LAYOUT
        ##########################  

        self.main_box = BoxLayout(orientation ='vertical')
        self.main_box.add_widget(self.header)
        self.main_box.add_widget(self.content)
        self.main_box.add_widget(self.autonomy_buttons_header)
        self.add_widget(self.main_box)

    def load_content(self):
        self.data_container = BoxLayout(orientation ='vertical', size_hint_x = 0.25)
        self.visual_container = BoxLayout(orientation ='vertical', size_hint_x = 0.75)

        self.state_container = BoxLayout(orientation ='vertical')
        self.position_container = BoxLayout(orientation ='vertical')
        self.console_container = BoxLayout(orientation ='horizontal', size_hint_y = 0.25)
        self.camera_container = BoxLayout(orientation ='horizontal', size_hint_y = 0.75)

        self.console = Label(text = "Initializing...", font_size = 25)
        self.console_container.add_widget(self.console)

        self.pos_title = Label(text = "DATA:", color = self.white, font_size=30, halign = "left")
        self.position_container.add_widget(self.pos_title)

        self.position_forks_data = Label(text = f"{self.STATE_MACHINE.get_forks_position_str()}", color = self.white)
        self.position_forks_data.font_size = 25
        self.position_container.add_widget(self.position_forks_data)

        self.position_forks_rel_data = Label(text = f"{self.STATE_MACHINE.get_forks_rel_position_str()}", color = self.white)
        self.position_forks_rel_data.font_size = 25
        self.position_container.add_widget(self.position_forks_rel_data)

        self.position_fork_pocket_error_data = Label(text = f"{self.STATE_MACHINE.get_fork_pocket_error_str()}", color = self.white)
        self.position_fork_pocket_error_data.font_size = 25
        self.position_container.add_widget(self.position_fork_pocket_error_data)

        self.states_title = Label(text = "STATES:", color = self.white, font_size=30, halign = "left")
        self.state_container.add_widget(self.states_title)
        
        self.last_state_label_data = Label(text = str(f"LAST = {self.STATE_MACHINE.last_state}"),
            color = self.white,
            font_size = 25
        )
        self.state_container.add_widget(self.last_state_label_data)

        self.curr_state_label_data = Label(text = str(f"CURRENT = {self.STATE_MACHINE.current_state}"), color = self.cyan)
        self.curr_state_label_data.font_size = 25
        self.state_container.add_widget(self.curr_state_label_data)

        self.next_state_label_data = Label(text = str(f"NEXT = {self.STATE_MACHINE.next_state}"), color = self.white)
        self.next_state_label_data.font_size = 25
        self.state_container.add_widget(self.next_state_label_data)

        self.data_container.add_widget(self.position_container)
        self.data_container.add_widget(self.state_container)

        self.visual_container.add_widget(self.camera_container)
        self.visual_container.add_widget(self.console_container)

        self.content.add_widget(self.data_container)
        self.content.add_widget(self.visual_container)

        # ERROR 
        if self.STATE_MACHINE.current_state == STATE_SPACE[0]:
            self.console.text = self.STATE_MACHINE.error_string
            return

        # MANUAL
        if self.STATE_MACHINE.current_state == STATE_SPACE[1]:
            self.console.text = "Choose enable Autonomy to continue..."
            return

        # AUTONOMY INIT
        elif self.STATE_MACHINE.current_state == STATE_SPACE[2]:
            ## USE CLOCK.SCHDULE AUTONOMY POP 
            return

        # PICK INIT
        elif self.STATE_MACHINE.current_state == STATE_SPACE[3]:
            self.console_container.add_widget(self.state_control_continue_button)
            self.console.text = "Pick Pallet Initializtion... Drive to Pallet and align facing it"
            return
        
        # PICK SELECT
        elif self.STATE_MACHINE.current_state == STATE_SPACE[4]:
            self.console.text = "Select a Pallet to Pick (Cannot be a pallet with 2 or more pallet above it)"
            return
                    
        # STATE_PICK_FORK_APPROX_ALIGN
        elif self.STATE_MACHINE.current_state == STATE_SPACE[5]:
            if self.STATE_MACHINE.fork_initialized:
                self.console.text = "Forks initialized..."
            else:
                self.console.text = "Waiting for fork initialization..."
            return

        # STATE_PICK_POCKET_DETECT_ENABLE
        elif self.STATE_MACHINE.current_state == STATE_SPACE[6]:
            if self.STATE_MACHINE.pocket_detected:
                self.console.text = "Found a Pocket!"
            else:
                self.console.text = "No pocket detected"
            return

        # STATE_PICK_CLOSEDLOOP_FORK_ENABLE
        elif self.STATE_MACHINE.current_state == STATE_SPACE[7]:
            self.console.text = "SLOWLY: Drive forward"
            return

        # STATE_PICK_POCKET_DETECT_CLOSEDLOOP_DISABLE
        elif self.STATE_MACHINE.current_state == STATE_SPACE[8]:
            self.console.text = "SLOWLY: Drive forward (Wait for forks to be fully inserted)"
            return

        # STATE_PICK_LIFT_PALLET
        elif self.STATE_MACHINE.current_state == STATE_SPACE[9]:
            self.console.text = "Lifting pallet..."
            return
        
        # STATE_PLACEGND_INIT
        elif self.STATE_MACHINE.current_state == STATE_SPACE[10]:
            self.console.text = "Initializtion..."
            return

        # STATE_PLACEGND_PLACE_PALLET
        elif self.STATE_MACHINE.current_state == STATE_SPACE[11]:
            self.console.text = "Placing Pallet on the Ground..."
            return
        
        # STATE_PLACESTACK_INIT
        elif self.STATE_MACHINE.current_state == STATE_SPACE[12]:
            self.console.text = "Initializtion..."
            return
        
        # STATE_PLACESTACK_FORK_APPROX_ALIGN
        elif self.STATE_MACHINE.current_state == STATE_SPACE[13]:
            self.console.text = "Initializing Forks..."
            return
        
        # STATE_PLACESTACK_PALLET_ALIGN
        elif self.STATE_MACHINE.current_state == STATE_SPACE[14]:
            self.console.text = "Checking for aligned pallet"
            return
        
        # STATE_PLACESTACK_PLACE_PALLET
        elif self.STATE_MACHINE.current_state == STATE_SPACE[15]:
            self.console.text = "Placing pallet on stack..."
            return

    def check_for_new_state(self, dt):
        if self.STATE_MACHINE.new_state:
            self.STATE_MACHINE.new_state = False
            self.remove_widget(self.main_box)
            self.load_screen()

    def check_for_pallet_popup(self, dt):
        if self.STATE_MACHINE.flag_ask_for_pallet_selection == True or self.STATE_MACHINE.flag_ask_pallet_again == True:

            self.STATE_MACHINE.flag_ask_for_pallet_selection = False
            self.STATE_MACHINE.flag_ask_pallet_again = False

            self.choose_pallet_mode = BoxLayout(orientation='vertical', size=(self.popup_frame_width, self.popup_frame_height), size_hint=(1, 1))

            self.choose_pallet_label = Label(text = "Select a pallet", font_size = 40)
            self.choose_pallet_mode.add_widget(self.choose_pallet_label)

            for i in reversed(range(self.STATE_MACHINE.pallet_stack_count)):
                # print(i)
                b = Button(text=f"Pallet {i}", font_size = 50)
                b.bind(on_press=self.pick_select_callback)
                self.choose_pallet_mode.add_widget(b)

            c = Button(text = "Close", font_size = 50)
            c.bind(on_press=self.close_pick_select_callback)
            c.background_color = self.red
            c.color = self.orange
            self.choose_pallet_mode.add_widget(c)

            self.choose_pallet_popup = Popup(title ='Select Pallet',
                title_size = '20sp',
                content = self.choose_pallet_mode,
                auto_dismiss=False,
                size=(self.popup_width, self.popup_height),
                size_hint=(None, None)
            )
            self.choose_pallet_popup.open()

    def check_for_mode_popup(self, dt):
        if self.STATE_MACHINE.flag_ask_for_mode_selection == True or self.STATE_MACHINE.flag_ask_mode_again == True:

            self.STATE_MACHINE.flag_ask_for_mode_selection = False
            self.STATE_MACHINE.flag_ask_mode_again = False

            self.choose_autonomy_mode = BoxLayout(orientation='vertical', size=(self.popup_frame_width, self.popup_frame_height), size_hint=(1, 1))

            self.autonomy_mode_label = Label(text = "Choose an autonomy mode!", font_size = 40)
            self.choose_autonomy_mode.add_widget(self.autonomy_mode_label)

            self.close_selection = Button(font_size = 50)
            self.close_selection.text = "Close"
            self.close_selection.background_color = self.red
            self.close_selection.color = self.orange
            self.close_selection.bind(on_press = self.close_mode_select_callback)

            self.pick_button = Button(font_size = 50)
            self.pick_button.text = "Pick Pallet"
            self.pick_button.bind(on_press = self.mode_select_callback)
            self.choose_autonomy_mode.add_widget(self.pick_button)

            self.place_ground_button = Button(font_size = 50)
            self.place_ground_button.text = "Place on Ground"
            self.place_ground_button.bind(on_press = self.mode_select_callback)
            self.choose_autonomy_mode.add_widget(self.place_ground_button)

            self.place_stack_button = Button(font_size = 50)
            self.place_stack_button.text = "Place on Stack"
            self.place_stack_button.bind(on_press = self.mode_select_callback)
            self.choose_autonomy_mode.add_widget(self.place_stack_button)

            self.choose_autonomy_mode.add_widget(self.close_selection)

            self.autonomy_mode_popup = Popup(title ='Select Event',
                title_size = '20sp',
                content = self.choose_autonomy_mode,
                auto_dismiss=False,
                size=(self.popup_width, self.popup_height),
                size_hint=(None, None)
            )
            self.autonomy_mode_popup.open()

    def settings_button_callback(self,*args):
        # self.manager.current = 'settings'
        pass

    def close_button_callback(self, event):  
        # try:
        #     for key, value in self.process_dict.items():
        #       self.process_dict[key].send_signal(subprocess.signal.SIGINT) 
        # except:
        #     print("Nothing to Kill!") 
        # self.STATE_MACHINE.kill()          
        App.get_running_app().stop()
        Window.close()

    def close_mode_select_callback(self, event):
        self.select_button = Button(text = "Select Mode", font_size = 50)
        self.select_button.bind(on_press = self.select_mode_again_callback)
        self.select_button.background_color = self.red
        self.select_button.color = self.orange
        self.autonomy_buttons_header.add_widget(self.select_button)
        self.autonomy_mode_popup.dismiss()
        return

    def close_pick_select_callback(self, event):
        self.select_button = Button(text = "Select Pallet", font_size = 50)
        self.select_button.bind(on_press = self.select_pallet_again_callback)
        self.select_button.background_color = self.red
        self.select_button.color = self.orange
        self.autonomy_buttons_header.add_widget(self.select_button)
        self.choose_pallet_popup.dismiss()
        return

    def pick_select_callback(self, event):
        num_stack = self.STATE_MACHINE.pallet_stack_count
        max_valid = num_stack-3

        val = int(str(event.text).split(" ")[1])

        if val <= max_valid:
            try:
                self.choose_pallet_label.text = "NOT VALID PALLET..."
                self.STATE_MACHINE.error_string = "NOT VALID PALLET..."
            except:
                print('ERROR')
        else:
            # self.STATE_MACHINE.flag_ask_for_pallet_selection = False
            self.STATE_MACHINE.user_selected_pallet = val
            self.STATE_MACHINE.flag_pallet_selected = True
            self.choose_pallet_popup.dismiss()

    def mode_select_callback(self, event):
        ''' Will switch from autonomy init to the various mode init states given the entry conditions are met'''
        print("MODE SELECT")
        if event.text == self.pick_button.text:
            self.STATE_MACHINE.flag_ask_for_mode_selection = False
            self.STATE_MACHINE.user_selected_mode = 1 # PICK
            self.STATE_MACHINE.flag_mode_selected = True

        elif event.text == self.place_ground_button.text:
            self.STATE_MACHINE.flag_ask_for_mode_selection = False
            self.STATE_MACHINE.user_selected_mode = 3 # PLACE ON GND
            self.STATE_MACHINE.flag_mode_selected = True

        elif event.text == self.place_stack_button.text:
            self.STATE_MACHINE.flag_ask_for_mode_selection = False
            self.STATE_MACHINE.user_selected_mode = 2 # PLACE ON STACK
            self.STATE_MACHINE.flag_mode_selected = True

        self.autonomy_mode_popup.dismiss()

    def select_mode_again_callback(self, event):
        self.autonomy_buttons_header.remove_widget(self.select_button)
        self.STATE_MACHINE.flag_ask_mode_again = True

    def select_pallet_again_callback(self, event):
        self.autonomy_buttons_header.remove_widget(self.select_button)
        self.STATE_MACHINE.flag_ask_pallet_again = True

    def enable_autonomy_callback(self, event):
        ''' Enable autonomy using ros service and show on console '''
        self.STATE_MACHINE.enable_autonomy()
        # Clock.schedule_once(, 0)
        
    def disable_autonomy_callback(self, event):
        ''' Disable autonomy using ros service and show on console '''
        self.STATE_MACHINE.disable_autonomy()
        # Clock.schedule_once(, 0)

class forklift_guiApp(App):
    def build(self):
        sm = ScreenManagement(transition=FadeTransition())
        Window.clearcolor = (0, 0, 0, 1)
        Window.size = (1920, 1080)
        sm.add_widget(MainWindow(name='main'))
        return sm

    def on_stop(self):
        pass    

    def resourcePath():
        if hasattr(sys, '_MEIPASS'):
            return os.path.join(sys._MEIPASS)
        return os.path.join(os.path.abspath("."))      

if __name__ == "__main__":
    forklift_guiApp().run()
