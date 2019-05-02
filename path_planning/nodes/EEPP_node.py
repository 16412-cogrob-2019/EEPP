#!/usr/bin/env python

# imports
import sys
import numpy as np
import json

# ros imports
import rospy
from std_msgs.msg import String

#path planning imports
from mypackage import plan_path


class EEPP_node:
    def __init__(self):
		self.Algorithm = "A*"
		self.map = None
        # subscribers
        self.maas_sub = rospy.Subscriber("/maas/poi_data", String, self.maas_callback)
        self.map_sub = rospy.Subscriber("/core/map", String, self.map_callback)

        # publisher
        self.mcts_pub = rospy.Publisher("/mcts/path_data", String, queue_size = 10) # jsonified data



############################# Subscriber Callback functions ####################
    def maas_callback(self, data):
		print("receieved data from maas module")

		if self.map == None:
			print("Error: Map has not yet been recieved!")
			return

		message = json.loads(data.data)

		paths, costs = plan_path(self, map, alg, alpha)
		data.paths = path
		data.costs = cost
		self.publish_paths(data)

	def map_callback(self, data):
		self.map = data

############################# Publisher functions ##############################
    def publish_paths(self, data):
	    self.mcts_pub.publish(json.dumps(data))




############################# Main #############################################
def main():
    # init ros node
    rospy.init_node('EEPP', anonymous = True)

    # class instance
    MAAS_instance = EEPP_node()

    # create ros loop
    pub_rate = 1 # hertz
    rate = rospy.Rate(pub_rate)
    while (not rospy.is_shutdown()):
        # # do some stuff if necessary
		# print("Publish Points of Interest")
		# MAAS_instance.publish_points()

        # ros sleep (sleep to maintain loop rate)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
