#!/usr/bin/env python3
import geometry_msgs.msg
import rospy
import roslib
import tf_conversions
import tf2_ros
from std_msgs.msg import String

from scipy.optimize import minimize as fmin

# /* estimated camera2 pose */
m_rotx = -0.653
m_roty = 0.271
m_rotz = -0.271
m_rotw = 0.653
m_tranx = 0.073
m_trany = 0.073
m_tranz = 0


class DemoNode:
    def __init__(self):
        print("entering constructor")
        self.counter = 1
        print("creating a publisher")
        self.timer = rospy.Timer(rospy.Duration(1), self.optimize_position)
        print("timer called")

        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.transformation_stamped = ()

    def optimize_position(self, timer):
        print("callback")
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "uav1/basler_stereopair/base"
        t.child_frame_id = "uav1/basler_left_optical"
        t.transform.rotation.w = m_rotw
        t.transform.rotation.x = m_rotx
        t.transform.rotation.y = m_roty
        t.transform.rotation.z = m_rotz
        t.transform.translation.x = m_tranx
        t.transform.translation.y = m_trany
        t.transform.translation.z = m_tranz
        br.sendTransform(t)

        print("sent pose")
        print("[basler calibration python] looking for transformation...")
        try:
            self.transformation_stamped = self.buffer.lookup_transform('uav1/basler_right_optical/tag_1',
                                                                       'uav1/basler_left_optical/tag_1',
                                                                       rospy.Time(), rospy.Duration(2))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
            print("[basler calibration python] no transformation found")

        print(self.transformation_stamped)



if __name__ == "__main__":
    rospy.init_node('python_calibration')
    try:
        DemoNode()
        print("entering Try")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("exception thrown")
