#!/usr/bin/env python

"""
Gets the position of the centre and it commands to steer the wheels
Subscribes to
    /line/point_line
    /pan_and_tilt/yaw_joint_position_controller/command

Publishes commands to
    /cmd_vel
    /button
"""

import time
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Float64
import math


const_steering = 0.0
const_throttle = 0.15


class ChaseLine:

    def __init__(self):

        rospy.loginfo("Open the Node")
        rospy.init_node('chase_line')

        self.sub_center = rospy.Subscriber("/line/point_line", Point, self.update_message)
        self.sub_angle = rospy.Subscriber('/pan_and_tilt/yaw_joint_position_controller/command',
                                          Float64, self.update_ang)
        rospy.loginfo("Subscribers set")

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=20, latch=True)
        self.pub_bool = rospy.Publisher('/button', Bool, queue_size=1)
        rospy.loginfo("Publisher set")

        self.throttle_action = 0.0
        self.brake_action = 0.0
        self.steer_action = 0.0

        self.center_x = 0.0
        self.center_y = 0.0
        self.center_z = 0.0

        self._message = Twist()
        # --- Set the time
        self.time_chase = time.time()
        self.lost_line = time.time()
        self.timeout_line = 1

        self._ang = Float64()

        self.mg = True

        self.msg = None

    def update_message(self, message):
        self.lost_line = time.time()
        self.mg = True
        self.center_x = message.x
        self.center_y = message.y
        self.center_z = message.z
        rospy.loginfo("Centres detected: %.1f  %.1f  %.1f" % (self.center_x, self.center_y, self.center_z))
        return self.lost_line

    def update_ang(self, mgs):
        # self.lost_line = time.time()
        self._ang = mgs
        print(self._ang)
        rospy.loginfo("Angle... " + str(self._ang))
        return self._ang

    @property
    def is_following(self):
        # print(time.time() - self.lost_line)
        # print(time.time() - self.lost_line < self.timeout_line)
        return (time.time() - self.lost_line) < self.timeout_line

    def get_control_action(self):
        """
        Based on the current ranges, calculate the command
        """
        steer_action = 0.0
        brake_action = 0.0
        throttle_action = 0.0
        r_wheel = 0.06

        self.mg = False
        rospy.loginfo("Centre detected: %.3f  %.3f  %.3f" % (self.center_x, self.center_y, self.center_z))

        # if self.is_following:# and 0.1 < self.center_x < 0.1:
        if self.center_x != 0:
            self.mg = True
            rospy.loginfo("YYYEEEESSS")
            print(float(self._ang.data))
            # throttle_action = const_throttle  #* self.center_y

            if -0.1999 < self.center_x < 0.1999:
                throttle_action = const_throttle  # * self.center_y
                self._message.linear.x = 0.08
                self.pub_twist.publish(self._message)

            if 0.2 < self.center_x < 0.9 and self._ang.data < 0:
                self._message.angular.z = 0.2
                turn_time = 10 * ((2 * r_wheel) * (math.sin(math.radians(self._ang.data))) / self._message.angular.z * -1)
                self.pub_twist.publish(self._message)
                rospy.loginfo("+++++++++++++++======++++++++++++++")
                time.sleep(turn_time)

            if -0.99 < self.center_x < -0.2 and self._ang.data > 0:
                self._message.angular.z = -0.2
                turn_time = 10 * ((2 * r_wheel) * (math.sin(math.radians(self._ang.data))) / self._message.angular.z * -1)
                self.pub_twist.publish(self._message)
                rospy.loginfo("--------------======------------")
                time.sleep(turn_time)

        # if not self.is_following:
        if self.center_x == 0:
            self.mg = False
            print(self._ang.data)
            self.msg = self.mg
            throttle_action = const_throttle  # * self.center_y
            self._message.linear.x = 0.0
            self.pub_twist.publish(self._message)

        #-- update the message
        self._message.linear.x = throttle_action
        self._message.linear.y = brake_action
        self._message.angular.z = steer_action

        self.pub_bool.publish(self.mg)
        rospy.loginfo("Following... " + str(self.mg))

        return throttle_action, brake_action, steer_action

    def run(self):

        while not rospy.is_shutdown():
            # -- Get the control action
            self.get_control_action()


if __name__ == "__main__":
    chase_line = ChaseLine()
    chase_line.run()
    try:
        rospy.spin()
        rospy.Rate(10)
    except KeyboardInterrupt:
        print("Shutting down")
