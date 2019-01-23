import rospy
import tf2_ros
from cairo_robot_interface.srv import TransformLookup, TransformLookupResponse


class TransformLookupService():

    def __init__(self, service_name):
        self.buffer = tf2_ros.Buffer()
        self.service_name = service_name

    def lookup_transform(self, req):
        resp = TransformLookupResponse()
        try:
            resp.trasnform = self.buffer.lookup_transform(req.source_frame, req.target_frame, rospy.Time())
            return resp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            resp.error.error_string = e.message
            return resp

    def start_server(self):
        rospy.init_node(self.service_name)
        s = rospy.Service(self.service_name, TransformLookup, self.lookup_transform)
        rospy.loginfo("{} service running...".format(self.service_name))
        rospy.spin()