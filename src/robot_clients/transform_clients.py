import rospy
import tf2_ros
from robot_clients.abstract_clients import AbstractROSClient
from cairo_robot_interface.srv import TransformLookup, TransformLookupRequest
from geometry_msgs.msg import TransformStamped


class TransformLookupClient(AbstractROSClient):
    def __init__(self, service_name):
        self.ns = service_name
        self.service = rospy.ServiceProxy(self.ns, TransformLookup)
        rospy.loginfo("Connecting to Transform Lookup service.")
        try:
            rospy.wait_for_service(self.ns, 20)
            rospy.loginfo("Connected to Transform Lookup service")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
        except rospy.ROSException as e:
            rospy.logerr("ROS Exception, service call failed: %s" % (e,))

    def close(self):
        self.service.close()

    def call(self, source_frame, target_frame):

        """

        Parameters
        ----------

        Returns
        -------

        """

        req = TransformLookupRequest()
        req.source_frame = source_frame
        req.target_frame = target_frame

        try:
            rospy.wait_for_service(self.ns, 5.0)
            resp = self.service(req)
            if resp.error.error_string is not None:
                rospy.logwarn("Transform lookup failed: %s" % (e,))
                return TransformStamped()
            return resp.transform
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % (e,))
            return TransformStamped()
        except rospy.ROSException as e:
            rospy.logerr("ROS Exception, service call failed: %s" % (e,))
            return TransformStamped()

        if resp.error.error_string is not None:
            rospy.logwarn("Transform lookup failed: %s" % (e,))
            return TransformStamped()
