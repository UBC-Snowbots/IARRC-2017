#!/usr/bin/env python
# license removed for brevity

# import the necessary packages
from __future__ import print_function
from pyimagesearch.panorama import Stitcher
from imutils.video import VideoStream
import numpy as np
import datetime
import imutils
import time
import cv2

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# The following code attempts to stitch 3 images from 3 cameras, and would potentially be improved 
# to stitch an input number of images in the future

# Inialize the video streams and allow them to warmup
print("[INFO] starting cameras...")
midStream = VideoStream(src=2).start()
leftStream = VideoStream(src=3).start()
rightStream = VideoStream(src=1).start()
time.sleep(1.0)

# Initialize the image stitcher, and total number of frames read
stitcher = Stitcher()
total = 0

# declares that my node is publishing to the 'ImgStitcher' topic, using type Image; 
# queue_size = 1 limits the queued msg number to be less than 1
# Aka only publish the most recent image
ImgStitcher = rospy.Publisher('ImgStitcher', Image, queue_size=1) 

# Initialize the publisher node called 'ImgPublisher'
rospy.init_node('ImgPublisher')

rate = rospy.Rate(10) # 10hz

# loop over frames from the video streams
while not rospy.is_shutdown():
	# grab the frames from their respective video streams
	left = leftStream.read()
        mid = midStream.read()
	right = rightStream.read()

	# resize the frames
	left = imutils.resize(left, width=400)
	right = imutils.resize(right, width=400)
        mid = imutils.resize(mid, width=400)

	# stitch the frames together to form the panorama
	# IMPORTANT: you might have to change this line of code
	# depending on how your cameras are oriented; frames
	# should be supplied in left-to-right order
	result_3 = stitcher.stitch([left, mid, right])

	# no homograpy could be computed
	if result_3 is None:
		print("[INFO] homography could not be computed")
		break

	#Convert OpenCV Mat to ROS Image type
	image_cv = CvBridge().cv2_to_imgmsg(result_3, "bgr8")

	#Publish ROS image
	try: 
		ImgStitcher.publish(image_cv)
        except CvBridgeError as e:
		print(e)

	rate.sleep()

	# show the output images
	cv2.imshow("result_3", result_3)
	cv2.imshow("Middle Frame", mid)
	cv2.imshow("Left Frame", left)
	cv2.imshow("Right Frame", right)
	key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
print("[INFO] cleaning up...")
cv2.destroyAllWindows()
leftStream.stop()
rightStream.stop()
