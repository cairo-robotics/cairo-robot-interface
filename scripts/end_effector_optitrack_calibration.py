#!/usr/bin/env python3
import rospy

# to get commandline arguments
import sys

# because of transformations
import tf
from pyquaternion import Quaternion

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':

        rospy.init_node('my_static_tf2_broadcaster')
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()

        vrpn_screen_pos = [-0.15093, 0.04005, 0.65956]
        vrpn_screen_quat = Quaternion(0.0020944, 0.11453, -0.0023762, 0.99341)

        screen_pos = [0.030014, 0.00029766, 0.48111]
        screen_quat = Quaternion(0.50337, 0.49723, 0.49613, 0.50323)
        relative_quat = Quaternion(0.0020937, 0.11453, -0.0023634, 0.99341)
        vrpn_world_quat = Quaternion(0, 0, 0, 1)

        new_quat = vrpn_world_quat * relative_quat * screen_quat

        quat_trans = vrpn_screen_quat.inverse * screen_quat
        pos_trans = [screen_pos[0] - vrpn_screen_pos[0],
                     screen_pos[1] - vrpn_screen_pos[1],
                     screen_pos[2] - vrpn_screen_pos[2]]

        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = '/world'
        static_transformStamped.child_frame_id = '/optitrack_world'
        # static_transformStamped.transform.translation.x = pos_trans[0]
        # static_transformStamped.transform.translation.y = pos_trans[1]
        # static_transformStamped.transform.translation.z = pos_trans[2]

        # static_transformStamped.transform.rotation.x = quat_trans[0]
        # static_transformStamped.transform.rotation.y = quat_trans[1]
        # static_transformStamped.transform.rotation.z = quat_trans[2]
        # static_transformStamped.transform.rotation.w = quat_trans[3]

        # inv_quat = 0.0021055, 0.11456, -0.0023711, 0.99341
        pose = [ 0.05271208, -0.0038469 , -0.05853307, 0.00858905, -0.00393499,  0.00110157,  0.99995476]

        # pose = [0, 0, 0, 0, 0, 0, 1]
        # Align world frames
        static_transformStamped.transform.translation.x = pose[0]
        static_transformStamped.transform.translation.y = pose[1]
        static_transformStamped.transform.translation.z = pose[2]
        static_transformStamped.transform.rotation.x = pose[3]
        static_transformStamped.transform.rotation.y = pose[4]
        static_transformStamped.transform.rotation.z = pose[5]
        static_transformStamped.transform.rotation.w = pose[6]

        # broadcaster.sendTransform(static_transformStamped)

        # Translate by relative distance to world frame.
        # static_transformStamped.transform.translation.x = 0
        # static_transformStamped.transform.translation.y = 0
        # static_transformStamped.transform.translation.z = 0
        # static_transformStamped.transform.rotation.x = 0
        # static_transformStamped.transform.rotation.y = 0
        # static_transformStamped.transform.rotation.z = 0
        # static_transformStamped.transform.rotation.w = 1

        # static_transformStamped.transform.translation.x = pos_trans[0]
        # static_transformStamped.transform.translation.y = pos_trans[1]
        # static_transformStamped.transform.translation.z = pos_trans[2]

        # static_transformStamped.transform.rotation.x = new_quat[0]
        # static_transformStamped.transform.rotation.y = new_quat[1]
        # static_transformStamped.transform.rotation.z = new_quat[2]
        # static_transformStamped.transform.rotation.w = new_quat[3]

        broadcaster.sendTransform(static_transformStamped)
        rospy.spin()
