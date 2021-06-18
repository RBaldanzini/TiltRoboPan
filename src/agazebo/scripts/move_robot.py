#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist


class MoveRosBots(object):

    def __init__(self):
        pass

    @staticmethod
    def find_line(cmd_vel):
        cmd_vel.angular.z = 0.0
        cmd_vel.linear.x = 0.5


if __name__ == '__main__':
    moverosbots_object = MoveRosBots()
    twist_object = Twist()