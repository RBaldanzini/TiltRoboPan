#!/usr/bin/env python

"""
Identify the centre of the detected line
Subscribes to
    /pan_and_tilt/raspicam_node/image_raw

Publishes commands to
    /line/image_line
    /line/point_line
"""


import rospy
import cv2
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from line_detector2 import line_detect


class LineFollower(object):

    def __init__(self, camera_topic="/pan_and_tilt/raspicam_node/image_raw", detection_window=None):
        # We check which OpenCV version is installed.
        (self.major, minor, _) = cv2.__version__.split(".")
        rospy.logwarn("OpenCV Version Installed==>" + str(self.major))
        self.detection_window = detection_window

        self._t0 = time.time()
        self.refer_point = Point()

        print(">> Publishing image to topic image_blob")
        print(">> Publishing position to topic point_line")
        self.image_pub = rospy.Publisher("/line/image_line", Image, queue_size=1)
        self.refer_pub = rospy.Publisher("/line/point_line", Point, queue_size=1)

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber(camera_topic, Image, self.callback)
        print("<< Subscribed to topic /pan_and_tilt/raspicam_node/image_raw")

    def callback(self, data):

        try:
            self.bridge_object.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        #cv_image = draw_window(cv_image, self.detection_window, line=1)
        x, y = line_detect(cv_image)
        #print(x, y, z)
        cv2.waitKey(1)
        try:
            self.image_pub.publish(self.bridge_object.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        if x and y:
            # self.msg = True
            self.refer_point.x = x
            self.refer_point.y = y
            # self.refer_point.z = z
            self.refer_pub.publish(self.refer_point)
            print("refer_point x, y, z", x, y)
            rospy.Rate(10)


        else:
            # self.msg = False
            self.refer_point.x = 0
            self.refer_point.y = 0
            # self.refer_point.z = z
            self.refer_pub.publish(self.refer_point)
            print("refer_point x, y", x, y)

        fps = 1.0 / (time.time() - self._t0)
        print(fps)


if __name__ == '__main__':

    rgb_to_track = [83, 83, 83]
    rospy.init_node('line_follower', anonymous=False)
    robot_mover = LineFollower()

    try:
        rospy.spin()
        # rospy.Rate(10)
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


