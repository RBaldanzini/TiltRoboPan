#!/usr/bin/python

import rospy
import math
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


r_wheel = 0.06

#
# def pan_and_tilt_search(self):
#
#     """
#     Topic Publisher
#     """
#     while not rospy.is_shutdown():
#
#         rospy.loginfo("TRYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYy")
#         for angle in range(-90, 90, 1):
#             print("Moving Yaw=" + str(angle))
#             yaw_in_radians = math.radians(angle)
#             pan_angle_msg = Float64()
#             pan_angle_msg.data = yaw_in_radians
#             # Publish Joint Position
#             self.pub_pan_position.publish(pan_angle_msg)
#             time.sleep(0.1)
#             # break
#         break


class PanTilt:

    def __init__(self):

        self.sub_center = rospy.Subscriber("/line/point_line", Point, self.update_message)

        rospy.init_node('is_line_following')

        self.sub_bool = rospy.Subscriber("/button", Bool, self.update_bool)

        self.pub_twist = rospy.Publisher("/cmd_vel", Twist, queue_size=20, latch=True)

        self.pub_pan_position = rospy.Publisher(
            '/pan_and_tilt/yaw_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_tilt_position = rospy.Publisher(
            '/pan_and_tilt/pitch_joint_position_controller/command',
            Float64,
            queue_size=1)

        self.center_x = 0.0
        self.center_y = 0.0
        self.center_z = 0.0

        self._bool = Bool()
        self._message = Twist()

    def update_message(self, message):
        # self.lost_line = time.time()
        #print(self.lost_line)
        self.center_x = message.x
        self.center_y = message.y
        self.center_z = message.z
        rospy.loginfo("Centres detected: %.1f  %.1f  %.1f" % (self.center_x, self.center_y, self.center_z))
        return self.center_x, self.center_y, self.center_z

    # @property
    def update_bool(self, mgs):
        # self.lost_line = time.time()
        self._bool = mgs
        # print(self._bool)
        return self._bool

    def pan_and_tilt_move(self):

        """
        Topic Publisher
        """
        while not rospy.is_shutdown():

            rospy.loginfo("NNNNOOOOOOOOOOOOOOO")

            while self._bool.data is False:
                print("bbbbbbbbbb")

                for angle in range(10, 90, 1):
                    print("Moving Yaw=" + str(angle))
                    yaw_in_radians = math.radians(angle)
                    pan_angle_msg = Float64()
                    pan_angle_msg.data = yaw_in_radians
                    # Publish Joint Position
                    self.pub_pan_position.publish(pan_angle_msg)
                    time.sleep(0.1)

                    if self.center_x:
                        # break
                        self.pub_pan_position.publish(0)
                        steer_action = -0.1
                        print("aaaaaaaaaaa")
                        self._message.angular.z = steer_action
                        rospy.loginfo("cmd_vel==" + str(self._message))
                        self.pub_twist.publish(self._message)

                        # angle_r = angle * math.radians(angle)
                        turn_time = 10 * ((2 * r_wheel) * (math.sin(math.radians(angle))) / steer_action * -1)
                        print(turn_time)
                        time.sleep(turn_time)
                        # if -30 < self.center_z < 30:
                        steer_action = 0.0
                        self._message.angular.z = steer_action
                        self.pub_twist.publish(self._message)
                        # break

                    # break
            while self._bool.data is True:
                print("cccccc")

        # if not self._bool:
        #     print("gggggggg")
        #     pass

                # for angle in range(90, -90, -1):
                #     print("Moving Yaw=" + str(angle))
                #     yaw_in_radians = math.radians(angle)
                #     pan_angle_msg = Float64()
                #     pan_angle_msg.data = yaw_in_radians
                #     # Publish Joint Position
                #     self.pub_pan_position.publish(pan_angle_msg)
                #     time.sleep(0.15)
                #     print(self.center_z)
                    # if not self.center_z:
                    #     steer_action = -0.2
                    #     throttle_action = 0.1
                    #     print("aaaaaaaaaaa")
                    #     self._message.linear.x = throttle_action
                    #     self._message.angular.z = steer_action
                    #     # rospy.loginfo("cmd_vel==" + str(self._message))
                    #     self.pub_twist.publish(self._message)


                    # break
                # break


if __name__ == "__main__":

    # while not rospy.is_shutdown():
    # rospy.Rate(10)
    # rospy.spin()
    pan_tilt = PanTilt()
    pan_tilt.pan_and_tilt_move()

    try:

        rospy.spin()
        rospy.Rate(10)

    except KeyboardInterrupt:
        print("Shutting down")
