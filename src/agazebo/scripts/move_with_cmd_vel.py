#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from motor_driver import MotorDriver

class RobotMover(object):

    def __init__(self):
        rospy.Subscriber("/myrobot/cmd_vel", Twist, self.cmd_vel_callback)
        self.motor_driver = MotorDriver()
                                         
#        rospy.wait_for_service('/raspicam_node/start_capture')

        rospy.loginfo("RobotMover Started...")

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        # Decide Speed
        self.motor_driver.set_cmd_vel(linear_speed, angular_speed)

    def listener(self):
        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('myrobot_cmd_vel_listener', anonymous=True)
    robot_mover = RobotMover()
    robot_mover.listener()
