#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from taller2.cfg import dynamic_paramConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {int_param}, {double_param},\
          {str_param}, {bool_param}, {size}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(dynamic_paramConfig, callback)
    rospy.spin()
