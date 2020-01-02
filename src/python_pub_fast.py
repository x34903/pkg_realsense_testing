#!/usr/bin/env python
import rospy # necessary for ROS
from std_msgs.msg import Float64MultiArray # for ROS message

import pyrealsense2 as rs # RealSense

import numpy as np 



def matrix_to_array(y1):#,y2): #converts list to matrix	
	# print('matrix_to_array()')

	nr1 = np.shape(y1)[0]
	nc1 = np.shape(y1)[1]
	# print(nr1)
	# print(nc1)
	# nr2 = np.shape(y2)[0]
	# nc2 = np.shape(y2)[1]

	m1 = np.transpose(y1).reshape(1,nc1*nr1)
	# m2 = np.transpose(y2).reshape(1,nc2*nr2)	

	t0 = rospy.get_time() 
	y0 = [t0,nr1,nc1]#,nr2,nc2]
	y0 = np.append(y0,m1)
	# y0 = np.append(y0,m2)

	y0 = list(y0)

	return y0



def python_ros_publisher():
	# pub = rospy.Publisher('chatter_from_python_multi', Float64MultiArray, queue_size=1)
	# y = Float64MultiArray()
	# # pub = rospy.Publisher('chatter', String, queue_size=10)
	# rospy.init_node('talker', anonymous=True)
	# rate = rospy.Rate(10) # 10hz

	# test = np.array([[1, 2], [3, 4], [5, 6]])

	while not rospy.is_shutdown():

		# Get frameset of color and depth
		frames = pipeline.wait_for_frames()
		# frames.get_depth_frame() is a 640x360 depth image

		# Align the depth frame to color frame
		aligned_frames = align.process(frames)

		# Get aligned frames
		aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
		# color_frame = aligned_frames.get_color_frame()

		# Validate that both frames are valid
		if not aligned_depth_frame: #or not color_frame:
		    continue

		depth_image = np.asanyarray(aligned_depth_frame.get_data())
		# color_image = np.asanyarray(color_frame.get_data())

		vertical_center_depth = depth_image[:,310:330]


		y1 = matrix_to_array(vertical_center_depth) #,y2) # m1 and m2 are two matrices, y is list       
		# y1 = matrix_to_array(depth_image) #,y2) # m1 and m2 are two matrices, y is list       
		y.data = y1
		# rospy.loginfo(y)
		pub.publish(y)

		# hello_str = "hello world %s" % rospy.get_time()
		# rospy.loginfo(hello_str)
		# pub.publish(hello_str)
		rate.sleep()






if __name__ == '__main__':

	##-----------------------------------------------------------------------
	## ROS SETUP

	pub = rospy.Publisher('chatter_from_python_multi', Float64MultiArray, queue_size=1)
	y = Float64MultiArray()

	rospy.init_node('python_ros_publisher', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	##-----------------------------------------------------------------------
	## REALSENSE SETUP

	# Create a RealSense pipeline
	pipeline = rs.pipeline()

	#Create a config and configure the pipeline to stream
	#  different resolutions of color and depth streams
	config = rs.config()
	config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
	config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	# Start RealSense streaming
	profile = pipeline.start(config)

	# Getting the depth sensor's depth scale (see rs-align example for explanation)
	depth_sensor = profile.get_device().first_depth_sensor()
	depth_scale = depth_sensor.get_depth_scale()
	print("Depth Scale is: " , depth_scale)

	# We will be removing the background of objects more than
	#  clipping_distance_in_meters meters away
	clipping_distance_in_meters = 1 #1 meter
	clipping_distance = clipping_distance_in_meters / depth_scale

	# Create an align object
	# rs.align allows us to perform alignment of depth frames to others frames
	# The "align_to" is the stream type to which we plan to align depth frames.
	align_to = rs.stream.color
	align = rs.align(align_to)


	##-----------------------------------------------------------------------
	## READY TO RUN THE REST

	# some matrix
	# test = np.array([[1, 2], [3, 4], [5, 6]])

	print('python ros node began successfully. Run MATLAB now.')

	try:
		python_ros_publisher()
	except rospy.ROSInterruptException:
		pass

	# INITIATE PYTHON ROS NODE
	# rospy.init_node('pyros_node', anonymous=True)

	# # SUBSCRIBER SETUP :: FROM ROS TO PYTHON 
	# rospy.Subscriber('chatter_from_matlab3', Float64MultiArray, callback_from_matlab)
	# x = Float64MultiArray()

	# PUBLISHER SETUP :: FROM PYTHON TO ROS
	# pub = rospy.Publisher('chatter_from_python_multi', Float64MultiArray, queue_size=1)
	# y = Float64MultiArray()



	# rospy.spin() # callback waits to be called until python ros node is killed
	# hint: this allows us to avoid running the script ad nausium in the background

