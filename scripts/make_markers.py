#!/usr/bin/env python
import rospy
from visualization_msgs.msg import *

# Initialize basic marker
def initMarker(ns, idNum, position, scale):
	marker = Marker()
	marker.action = Marker.ADD
	marker.header.frame_id = "map"
	marker.ns = ns
	marker.id = idNum
	marker.pose.position = position
	marker.pose.orientation.w = 1.0
	marker.color.a = 1.0
	marker.scale.x = scale
	marker.scale.y = scale
	marker.scale.z = scale
	return marker

# Make static posts for AprilTags
# Each marker should have a unique idNum
# Create position with Point(x_position, y_position, z_position)
# orientation_z defines the orientation of the tag around the z-axis
# tag_num is a number 1-5 specifying which mesh to use
def makePost(idNum, position, orientation_z, tag_num):
	post = initMarker("posts", idNum, position, 1.0)
	post.type = Marker.MESH_RESOURCE
	post.pose.orientation.z = orientation_z
	post.color.a = 0.0
	post.mesh_resource = "package://map_visualization_2/meshes/post/post{}.dae".format(tag_num)
	post.mesh_use_embedded_materials = True
	return post