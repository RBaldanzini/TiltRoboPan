#!/usr/bin/python

"""
Identify the centre of the detected line
Gets the position of the centre and it commands to steer the wheels
Subscribes to
    /line/point_line
    /button

Publishes commands to
    /pan_and_tilt/yaw_joint_position_controller/command
    /pan_and_tilt/pitch_joint_position_controller/command
"""

import rospy
import math
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class PanTilt:

    def __init__(self):

        rospy.init_node('is_line_following')

        self.pub_pan_position = rospy.Publisher(
            '/pan_and_tilt/yaw_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_tilt_position = rospy.Publisher(
            '/pan_and_tilt/pitch_joint_position_controller/command',
            Float64,
            queue_size=1)

        self.sub_center = rospy.Subscriber("/line/point_line", Point, self.update_message)
        self.sub_bool = rospy.Subscriber("/button", Bool, self.update_bool)
        rospy.loginfo("Subscribers set")

        self._bool = Bool()
        self._message = Twist()
        self.center_x = 0.0

    def update_bool(self, mgs):
        self._bool = mgs
        # print(self._bool)
        return self._bool

    def update_message(self, message):
        #print(self.lost_line)
        self.center_x = message.x
        rospy.loginfo("Centres detected: %.1f" % self.center_x)
        return self.center_x

    def pan_and_tilt_search_left(self):

        for angle in range(0, 85, 3):
            print("Moving Yaw=" + str(angle))
            yaw_in_radians = math.radians(angle)
            pan_angle_msg = Float64()
            pan_angle_msg.data = yaw_in_radians
            # Publish Joint Position
            self.pub_pan_position.publish(pan_angle_msg)
            time.sleep(0.3)
            if -0.4 < self.center_x < 0.4 and self._bool.data is True:
                rospy.loginfo("+++++++++++++++======++++++++++++++")
                self.pub_pan_position.publish(0)
                time.sleep(5)
                break

    def pan_and_tilt_search_right(self):

        for aangle in range(80, -80, -3):
            print("Moving Yaw=" + str(aangle))
            yaw_in_radians = math.radians(aangle)
            pan_angle_msg = Float64()
            pan_angle_msg.data = yaw_in_radians
            # Publish Joint Position
            self.pub_pan_position.publish(pan_angle_msg)
            time.sleep(0.3)
            if -0.4 < self.center_x < 0.4 and self._bool.data is True:
                rospy.loginfo("-----------===========____________")
                self.pub_pan_position.publish(0)
                time.sleep(5)
                break

    def run(self):

        while not rospy.is_shutdown():

            if self._bool.data is False:
                self.pan_and_tilt_search_left()

            if self._bool.data is False:
                self.pan_and_tilt_search_right()

            if self._bool.data is True:
                pass


if __name__ == "__main__":

    pan_tilt = PanTilt()
    pan_tilt.run()

    try:
        rospy.spin()
        rospy.Rate(5)

    except KeyboardInterrupt:
        print("Shutting down")
