#! /usr/bin/env python2
from collections import deque
import argparse
import imutils
import liboCams
import cv2 as cv
import numpy as np
import time
import sys

      
devpath = liboCams.FindCamera('oCam')
if devpath is None:
  exit()

test = liboCams.oCams(devpath, verbose=1)
fmtlist = test.GetFormatList()
test.Close()

test = liboCams.oCams(devpath, verbose=0)

print 'SET', 8, fmtlist[8]
test.Set(fmtlist[8])
name = test.GetName()
test.Start()

start_time = time.time()

frame_cnt = 0

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-b", "--buffer", type=int, default=32,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points
redLower = (0, 50, 50)
redUpper = (10, 255, 255)
pts = deque(maxlen=args["buffer"])

while True:
	frame = test.GetFrame()
	rgb = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
	src = rgb 
	#cd cv.imshow(test.cam.card, rgb)
	
	#----------------------------------RED BALL TRACKING----------------------------------
	# Check if image is loaded fine
	if src is None:
		print ('Error opening image!')
		print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
		exit()

	# resize the frame, blur it, and convert it to the HSV
	# color space
	src = imutils.resize(rgb, width=600)
	blurred = cv.GaussianBlur(src, (11, 11), 0)
	hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
	
	# construct a mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv.inRange(hsv, redLower, redUpper)
	mask = cv.erode(mask, None, iterations=2)
	mask = cv.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,
		cv.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use it to
		# compute the minimum enclosing circle and centroid
		c = max(cnts, key=cv.contourArea)
		((x, y), radius) = cv.minEnclosingCircle(c)
		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv.circle(src, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
			cv.circle(src, center, 5, (0, 0, 255), -1)
	
	# update the points queue
	pts.appendleft(center)

	# loop over the set of tracked points
	for i in range(1, len(pts)):
		# if either of the tracked points are None, ignore
		# them
		if pts[i - 1] is None or pts[i] is None:
			continue
		
		# otherwise, compute the thickness of the line and
		# draw the connecting lines
		thickness = int(np.sqrt(args["buffer"] / float(i + 1)) * 2.5)
		cv.line(src, pts[i - 1], pts[i], (0, 0, 255), thickness)
	
	# show the frame to our screen
	cv.imshow("Frame", cnts)
	char = cv.waitKey(1)
	if char == 27:
		break
	frame_cnt += 1

print 'Result Frame Per Second:', frame_cnt/(time.time()-start_time)
test.Stop()  
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()
