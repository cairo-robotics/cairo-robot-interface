import rospy
import tf2_ros
from cairo_robot_interface.srv import TransformLookup, TransformLookupResponse


class TransformLookupService():
    """
    Class that creates a ROS service to handle incoming calls to calculate
    transformations from one frame to another. 

    Attributes
    ----------
    buffer : tf2_ros.Buffer
        The tf2 Buffer that service to lookup transforms
    listener : tf2_ros.TransformListener
        The TransformListener object that consumes the buffer to update its state.
    service_name : str
        The ROS Service proxy object
    """
    def __init__(self, service_name):
        """
        Parameters
        ----------
        buffer : tf2_ros.Buffer
            The tf2 Buffer that service to lookup transforms
        listener : tf2_ros.TransformListener
            The TransformListener object that consumes the buffer to update its state.
        service_name : str
            The ROS Service proxy object
        """
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.service_name = service_name

    def _lookup_transform(self, req):
        """
        Function to lookup transform given a TransformLookupRequest

        Parameters
        ----------
        req : TransformLookupRequest
            The request for the TrasnformLookupService
        """
        resp = TransformLookupResponse()
        try:
            resp.transform = self.buffer.lookup_transform(req.source_frame, req.target_frame, rospy.Time())
            resp.error.error = 0
            return resp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            resp.error.error = 1
            if hasattr(e, 'message'):
                resp.error.error_string = e.message
            else:
                resp.error.error_string = str(type(e))
            return resp

    def start_server(self):
        """
        Initiates/starts the Transform lookup service.
        """
        s = rospy.Service(self.service_name, TransformLookup, self._lookup_transform)
        rospy.loginfo("{} service running...".format(self.service_name))
        rospy.spin()