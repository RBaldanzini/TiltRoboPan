#!/usr/bin/python
import sys
import rospy
from pan_and_tilt_control.msg import PanAndTilt


class PanAndTiltServer(object):
    """
    Class for using the Pan and Tilt, real or simulated.
    This allows us to launch always the same node, and just
    initialise the move PanAndTilt object for the Real pan and tilt or
    for the simulated version in Gazebo. This way we reuse code and
    we force both classes to be transparent and have the same interface.
    """

    def __init__(self):
        rospy.loginfo("Starting PanAndTiltServer...")

        # We Initialise the Real or Simulated Control object
        from pan_and_tilt_move_sim import PanAndTiltMoveSim
        self._pan_and_tilt_move_object = PanAndTiltMoveSim()
        rospy.loginfo("PanAndTiltServer SIMULATED___STARTED")

        pan_and_tilt_sub = rospy.Subscriber('/pan_and_tilt', PanAndTilt, self.pan_and_tilt_callback)

        rospy.loginfo("PanAndTiltServer...READY")

    def pan_and_tilt_callback(self, msg):
        """
        Topic Subscriber callback
        """

        rospy.logdebug("Received PandAndTilt New msg: " + str(msg))

        self._pan_and_tilt_move_object.move_to_pitch_yaw(yaw=msg.pan, pitch=msg.tilt)


if __name__ == "__main__":

    rospy.init_node('pan_and_tilt_server')
    machine = PanAndTiltServer()
    rospy.spin()
