#!/usr/bin/env python
from __future__ import print_function
import os
import subprocess

import rospkg

from kivy.app import App
from kivy.core.window import Window
from kivy.uix.screenmanager import ScreenManager, Screen, FadeTransition
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.button import Button
from kivy.app import App
from kivy.uix.image import Image
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label

class ScreenManagement(ScreenManager):
    def __init__(self, **kwargs):
        super(ScreenManagement, self).__init__(**kwargs)

''' LANDING PAGE WINDOW THAT WILL SHOW FIRST '''
class MainWindow(Screen):
    def __init__(self, **kwargs):
        super(MainWindow, self).__init__(**kwargs)

        # HEADER ELEMENTS
        ##########################
        rospy.loginfo("I will publish to the topic")
        self.rospack = rospkg.RosPack()
        package_path = self.rospack.get_path('forklift_gui')
        print(package_path)
        
        cyngn_logo_path = os.path.join(package_path, "cyngn_logo.png")
        back_icon_path = os.path.join(package_path,"back.png")
        close_icon_path = os.path.join(package_path,"close.png")
        settings_icon_path = os.path.join(package_path,"settings.png")
        
        self.header = BoxLayout(orientation ='horizontal',          size_hint_y = None,     height = 150)
        
        self.cyngn_logo = Image(source = cyngn_logo_path,    size_hint_y = 0.8,      allow_stretch = True)

        self.label = Label(     text=str("Cyngn Forklift GUI"),     font_size = '45dp',     size_hint = (1, 0.8))
        
        self.settings_button = Button(height = 100, width = 100,  size_hint = (None, None), background_normal = settings_icon_path )
        self.settings_button.bind(on_press = self.settings_button_callback)

        self.close_button = Button( height = 100, width = 100,  size_hint = (None, None), background_normal = close_icon_path )
        self.close_button.bind(on_press = self.close_button_callback)

        self.header.add_widget(self.cyngn_logo)
        self.header.add_widget(self.label)
        self.header.add_widget(self.settings_button)
        self.header.add_widget(self.close_button)
        
        # CREATING THE LAYOUT
        ##########################  
        self.main_box = BoxLayout(orientation ='vertical')
        self.main_box.add_widget(self.header)
        self.add_widget(self.main_box)
            
    def settings_button_callback(self,*args):
        # self.manager.current = 'settings'
        pass

    def close_button_callback(self, event):  
        try:
            for key, value in self.process_dict.items():
              self.process_dict[key].send_signal(subprocess.signal.SIGINT) 
        except:
            print("Nothing to Kill!")           
        App.get_running_app().stop()
        Window.close()

class forklift_guiApp(App):
    def build(self):
        rospy.init_node("forklift_gui", anonymous=True)
        rospy.loginfo("I will publish to the topic")
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