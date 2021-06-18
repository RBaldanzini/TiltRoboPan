#!/usr/bin/env python
import time
import math
import rospy
from std_msgs.msg import Float64


class PanAndTiltMoveSim(object):

    def __init__(self):

        rospy.loginfo("PanAndTiltMoveSim.....STARTING")

        self.pan_value = 0.0
        self.tilt_value = 0.0
        self.tilt_upper = 0.7
        self.tilt_lower = -0.7
        self.pan_upper = 1.57
        self.pan_lower = -1.57

        self.pub_pan_position = rospy.Publisher(
            '/pan_and_tilt/yaw_joint_position_controller/command',
            Float64,
            queue_size=1)
        self.pub_tilt_position = rospy.Publisher(
            '/pan_and_tilt/pitch_joint_position_controller/command',
            Float64,
            queue_size=1)

        self.PITCH_INIT_ANGLE = 0
        self.YAW_INIT_ANGLE = 0
        self.move_to_pitch_yaw(yaw=self.YAW_INIT_ANGLE,
                               pitch=self.PITCH_INIT_ANGLE)

        rospy.loginfo("PanAndTiltMoveSim.....STARTED...DONE")

    def __del__(self):
        print("\nCleaning PanAndTiltMove...")
        print("\nPanAndTiltMove END")


    def wait_publishers_to_be_ready(self):

        publishers_ready = False

        rate_wait = rospy.Rate(10)
        while not publishers_ready:
            pan_num = self.pub_pan_position.get_num_connections()
            tilt_num = self.pub_tilt_position.get_num_connections()
            publishers_ready = (pan_num > 0) and (tilt_num > 0)
            rate_wait.sleep()

    def update_pan(self,pan_angle):

        if self.pan_upper >= pan_angle >= self.pan_lower:
            self.pan_value = pan_angle
        elif pan_angle < self.pan_lower:
            self.pan_value = self.pan_lower
        else:
            self.pan_value = self.pan_upper

        rospy.logdebug("pan_value=" + str(self.pan_value))

    def update_tilt(self,tilt_angle):

        if self.tilt_upper >= tilt_angle >= self.tilt_lower:
            self.tilt_value = tilt_angle
        elif tilt_angle < self.tilt_lower:
            self.tilt_value = self.tilt_lower
        else:
            self.tilt_value = self.tilt_upper

        rospy.logdebug("tilt_value="+str(self.tilt_value))



    def move_to_pitch_yaw(self, yaw, pitch):
        rospy.logdebug("move_to_pitch_yaw=" + str(yaw) +","+str(pitch))
        self.wait_publishers_to_be_ready()
        self.move_yaw(yaw_angle=yaw)
        self.move_pitch(pitch_angle=pitch)

    def move_yaw(self, yaw_angle):
        # We convert the Degrees given into radians
        yaw_in_radians = math.radians(yaw_angle)
        pan_angle_msg = Float64()
        pan_angle_msg.data = yaw_in_radians
        # Publish Joint Position
        self.pub_pan_position.publish(pan_angle_msg)
        self.update_pan(yaw_in_radians)

    def move_pitch(self, pitch_angle):
        # We convert the Degrees given into radians
        pitch_in_radians = math.radians(pitch_angle)
        tilt_angle_msg = Float64()
        tilt_angle_msg.data = pitch_in_radians
        # Publish Joint Position
        self.pub_tilt_position.publish(tilt_angle_msg)
        self.update_tilt(pitch_in_radians)

    def yaw_range_test(self):
        """
        It tests the range of yaw servo once
        :return:
        """
        for angle in range(10,170,1):
            print("Moving Yaw="+str(angle))
            self.move_yaw(yaw_angle=angle)
            time.sleep(0.1)

        for angle in range(170,10,-1):
            print("Moving Yaw="+str(angle))
            self.move_yaw(yaw_angle=angle)
            time.sleep(0.1)


    def pitch_range_test(self):
        """
        It tests the range of pitch servo once
        :return:
        """
        for angle in range(10,170,1):
            print("Moving Pitch="+str(angle))
            self.move_pitch(pitch_angle=angle)
            time.sleep(0.1)

        for angle in range(170,10,-1):
            print("Moving Pitch="+str(angle))
            self.move_pitch(pitch_angle=angle)
            time.sleep(0.1)

    def input_yaw_test(self):

        while True:
            input_x = raw_input("Type Angle to move to")
            angle = int(input_x)
            print("Moving Yaw=" + str(angle))
            self.move_yaw(yaw_angle=angle)

    def input_pitch_test(self):

        while True:
            input_x = raw_input("Type Angle to move to")
            angle = int(input_x)
            print("Moving Pitch=" + str(angle))
            self.move_pitch(pitch_angle=angle)

    def yaw_pitch_circle_test(self, repetitions=1):

        # These values are base on Hardware observations where they are stable.
        max_range = 100
        min_range = 30
        period = 0.1
        increments = 10

        for num in range(repetitions):
            for angle in range(0,359,increments):
                pitch_angle = int((((math.sin(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
                yaw_angle = int((((math.cos(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)

                print("Moving Yaw="+str(yaw_angle))
                print("Moving Pitch=" + str(pitch_angle))

                self.move_pitch(pitch_angle)
                self.move_yaw(yaw_angle)
                time.sleep(period)

            for angle in range(0,359,-increments):
                pitch_angle = int((((math.sin(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)
                yaw_angle = int((((math.cos(math.radians(angle)) + 1.0) / 2.0) * (max_range - min_range)) + min_range)

                print("Moving Yaw="+str(yaw_angle))
                print("Moving Pitch=" + str(pitch_angle))

                self.move_pitch(pitch_angle)
                self.move_yaw(yaw_angle)
                time.sleep(period)

def CircleTest():
    pt_object = PanAndTiltMoveSim()
    # raw_input("Start Circle Test...Press Key")
    pt_object.yaw_pitch_circle_test(repetitions=3)


if __name__ == "__main__":
    rospy.init_node('pan_and_tilt_move_sim')
    CircleTest()



