#!/usr/bin/env python3.6


import rospy
from std_msgs.msg import Bool, Float64
from geometry_msgs.msg import Twist
from kivymd.app import MDApp
from kivy.lang import Builder
import math


class MainApp(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.screen = Builder.load_file('/home/renjk/ArchiBot_PandT_Camera/src/aguiapp/ros_gui.kv')

    def build(self):
        return self.screen

    def func(self, *args):
        print("pressed")

        # self.screen.ids.first_topic.text = 'button pressed'
        self.screen.ids.first_topic.text = 'button pressed'

        msg = True
        pub.publish(msg)

    def direction_function(self, mess):
        print(mess)

        message = Twist()
        message.linear.x = mess
        pub_twist.publish(message)


    def slider_func(self, slider_value):
        print(slider_value)

        yaw_in_radians = math.radians(slider_value)
        pan_angle_msg = Float64()
        pan_angle_msg.data = yaw_in_radians

        pub_pan_position.publish(pan_angle_msg)


if __name__ == '__main__':
    pub = rospy.Publisher('/bbutton', Bool, queue_size=1)
    pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=20, latch=True)
    pub_pan_position = rospy.Publisher('/pan_and_tilt/yaw_joint_position_controller/command', Float64, queue_size=1)

    rospy.init_node('simple_gui', anonymous=True)
    MainApp().run()

