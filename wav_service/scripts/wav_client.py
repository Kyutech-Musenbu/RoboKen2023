#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
from wav_service.srv import Voice

def voice_client(X):
    rospy.wait_for_service('wav_service')
    try:
        receive_int = rospy.ServiceProxy('wav_service', Voice)
        resp = receive_int(X)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('int_service_client')
    x = 3  # Example integer
    print("Requesting %s" % x)
    print("%s -> %s" % (x, voice_client(x)))