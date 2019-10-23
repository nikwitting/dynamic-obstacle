#!/usr/bin/env python

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String, Float64

class MyNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyNode, self).__init__(node_name=node_name)
        offset_mode = 0
        offset_mode = int(os.environ['OFFSET'])
        if offset_mode == 0:
            self.offset = 0.0
        elif offset_mode == 1:
            self.offset = 0.2175    # lane distance
        elif offset_mode == 2:
            self.offset = 0.115     # middle of the road
        else:
            self.offset = 0.0
        # construct publisher
        self.pub = rospy.Publisher('lane_controller_node/doffset', Float64, queue_size=1)

    def run(self):
        # publish message every 0.1 second
        rate = rospy.Rate(10) # 1Hz
        while not rospy.is_shutdown():
            message = self.offset
            #rospy.loginfo("Offset published: '%f'" % message)
            self.pub.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyNode(node_name='my_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()
