#! /usr/bin/env python2

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

#example code for camera control
#val = test.GetControl(ctrlist['Exposure (Absolute)'])
#test.SetControl(ctrlist['Exposure (Absolute)'], 2)

start_time = time.time()

frame_cnt = 0
while True:
	frame = test.GetFrame()
	rgb = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
	#cd cv.imshow(test.cam.card, rgb)
	
	#----------------------------------HOUGH CIRCLE TRANSFORM----------------------------------
	# Loads an image
	src = rgb
	
	# Check if image is loaded fine
	if src is None:
		print ('Error opening image!')
		print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
		exit()
	## [load]

	## [convert_to_gray]
	# Convert it to gray
	#gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
	## [convert_to_gray]

	## [reduce_noise]
	# Reduce the noise to avoid false circle detection
	noise = cv.medianBlur(src, 5)
	blur = cv.GaussianBlur(src, (5,5),0)
	## [reduce_noise]

	## [apply red threshold]
	red = blur[:, :, 2]
    # green = frame[:, :, 1]
    # blue = frame[:, :, 0]
	red_thresholded = cv.adaptiveThreshold(red, 120, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 79, 0)
    # _, green_thresholded = cv2.threshold(green, 120, 255, cv2.THRESH_BINARY)
    # _, blue_thresholded = cv2.threshold(blue, 120, 255, cv2.THRESH_BINARY)
    ## [apply red threshold]

	## [houghcircles]
	rows = red_thresholded.shape[0]
	circles = cv.HoughCircles(red_thresholded, cv.HOUGH_GRADIENT, 1, rows / 8,
							   param1=100, param2=30,
							   minRadius=10, maxRadius=100)
	## [houghcircles]

	## [draw]
	if circles is not None:
		circles = np.uint16(np.around(circles))
		for i in circles[0, :]:
			center = (i[0], i[1])
			# circle center
			cv.circle(src, center, 1, (0, 100, 100), 3)
			# circle outline
			radius = i[2]
			cv.circle(src, center, radius, (255, 0, 255), 3)
	## [draw]

	## [display]
	cv.imshow("detected circles", red)
	## [display]
	#---------------------------------------------------------------------------------------------------------
	
	char = cv.waitKey(1)
	if char == 27:
		break
	frame_cnt += 1

print 'Result Frame Per Second:', frame_cnt/(time.time()-start_time)
test.Stop()  
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()
