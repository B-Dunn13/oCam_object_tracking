import imutils
import liboCams
import cv2 as cv
import numpy as np
import time
import sys

# find the oCam
devpath = liboCams.FindCamera('oCam')
if devpath is None:
  exit()

test = liboCams.oCams(devpath, verbose = 1)

# get the format list for the oCam
print 'Format List'
fmtlist = test.GetFormatList()
for fmt in fmtlist:
  print '\t', fmt

# set the format for the oCam (8-bit Bayer GRGR/BGBG,
# 640L, 480L, 30)
print 'SET', 10, fmtlist[10]
test.Set(fmtlist[10])
name = test.GetName()

# get the current camera parameters and set new
# camera prameters before recording
print 'Control List'
ctrlist = test.GetControlList()
for key in ctrlist:
  print '\t', key, '\tID:', ctrlist[key]
  print test.GetControl(ctrlist[key])
  if key == "White Balance Red Component":
    test.SetControl(ctrlist[key], 104)
  elif key == "White Balance Blue Component":
    test.SetControl(ctrlist[key], 250)
  elif key == "Gain":
    test.SetControl(ctrlist[key], 64)
  elif key == "Exposure (Absolute)":
    test.SetControl(ctrlist[key], 128)
  elif key == "Exposure, Auto":
    test.SetControl(ctrlist[key], 3)

test.Start()

start_time = time.time()

frame_cnt = 0

#----------------------------------RED BALL DETECTION----------------------------------
# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points. Must start before the while
# loop for the previous centroid pts to show.
redLower = (0, 50, 50)
redUpper = (10, 255, 255)

while True:
	frame = test.GetFrame()
	BGR = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
	src = BGR 
	
	# Check if image is loaded fine
	if src is None:
		print ('Error opening image!')
		print ('Usage: simple_code.py [oCam -- default ' + default_file + '] \n')
		exit()
	
	# resize the frame, blur it, and convert it to the HSV color space
	# src = imutils.resize(BGR, width = 600)
	blur1 = cv.medianBlur(src, 5)
	blur2 = cv.GaussianBlur(blur1, (11, 11), 0)
	hsv = cv.cvtColor(blur2, cv.COLOR_BGR2HSV)

	# construct a mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv.inRange(hsv, redLower, redUpper)
	mask = cv.erode(mask, None, iterations = 2)
	mask = cv.dilate(mask, None, iterations = 2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use it to
		# compute the bounding rectangle and centroid
		c = max(cnts, key = cv.contourArea)
		x, y, w, h = cv.boundingRect(c)
		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
		# draw the bounding box and centroid on the frame,
		# then update the list of tracked points
		cv.rectangle(src, (x, y), (x + w, y + h), (255, 255, 255), 2)
		cv.circle(src, center, 5, (0, 0, 255), -1)

	print 'Result Frame Per Second:', frame_cnt / (time.time() - start_time)
	print 'Bounding Box Points:', 'Top Left X:', x, 'Top Left Y:', y,\
	'Bottom Right X:', x + w, 'Bottom Right Y:', y + h

	# show the frame to our screen
	cv.imshow("Frame", src)
	char = cv.waitKey(1)
	if char == 27:
		break
	frame_cnt += 1
#----------------------------------RED BALL DETECTION----------------------------------
test.Stop()  
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()
