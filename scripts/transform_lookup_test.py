#!/usr/bin/env python

from robot_clients.transform_clients import TransformLookupClient

if __name__ == "__main__":
    service = TransformLookupClient(service_name='transform_lookup_service')
    print(service.call('base', 'right_gripper_tip'))