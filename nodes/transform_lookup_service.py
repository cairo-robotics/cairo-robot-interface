#!/usr/bin/env python

from robot_servers.transform_servers import TransformLookupService
import rospy

if __name__ == "__main__":
    rospy.init_node("transform_lookup_server")
    tls = TransformLookupService(service_name="transform_lookup_service")
    tls.start_server()