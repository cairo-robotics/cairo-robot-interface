#!/usr/bin/env python  
import roslib
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
import pyquaternion as py
import numpy

if __name__ == '__main__':
    rospy.init_node('vrpn_static_tf2_broadcaster')
    world_quat = py.Quaternion([0, 0, 0, 1])
    rob_quat = py.Quaternion([0.70551, -0.039878, -0.040895, 0.70639])
    opt_quat = py.Quaternion([0.013246, 0.00031744, 0.73941, 0.67312])
    quat = opt_quat.inverse * rob_quat
    optitrack_screen_pos = [-0.095943, 0.042206, 0.66858]
    inv_optitrack_screen_pos = [-x for x in optitrack_screen_pos]
    curr_screen_pos = [0.023908, 0.018003, 0.48112]
    # trans = curr_screen_pos
    trans = [sum(x) for x in zip(optitrack_screen_pos, curr_screen_pos)]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base"
    t.child_frame_id = "optitrack_world"
    t.transform.translation.x = 0
    t.transform.translation.y = 0
    t.transform.translation.z = 0
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    broadcaster.sendTransform(t)
    rospy.spin()