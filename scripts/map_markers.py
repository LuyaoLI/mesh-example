#!/usr/bin/env python
import rospy
import time
from make_markers import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point

if __name__=="__main__":
	rospy.init_node("map_markers")
	pubMarkers = rospy.Publisher("markers", MarkerArray, queue_size=100)
	
	# Marker array for everything
	markerArray = MarkerArray()
	# Create AprilTags
	# makePost function is found in make_markers.py
	markerArray.markers.append(makePost(8, Point(0.0, 0.0, 0.0), 1.0, 1))

	rate = rospy.Rate(15)
	while not rospy.is_shutdown():
		pubMarkers.publish(markerArray)
		rate.sleep()
	rospy.spin()