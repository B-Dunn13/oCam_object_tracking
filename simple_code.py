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
print 'SET', 8, fmtlist[8]
test.Set(fmtlist[8])

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

#----------------------------------OBJECT DETECTION----------------------------------
# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points. Must start before the while
# loop for the previous centroid pts to show.
GreenLower = (29, 86, 6)
GreenUpper = (64, 255, 255)

while True:
	frame = test.GetFrame()
	BGR = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
	src = BGR 
	
	# Check if image is loaded fine
	if src is None:
		print ('Error opening image!')
		print ('Usage: simple_code.py [oCam -- default ' + default_file + '] \n')
		exit()
	
	# blur the frame and convert it to the HSV color space
	src = imutils.resize(src, width=480)
	blur = cv.medianBlur(src, 3)
	blur = cv.GaussianBlur(blur, (11, 11), 0)
	hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

	# # construct a mask for the color "red", Lower mask (0-10)
	# redLower = np.array([0, 50, 50])
	# redUpper = np.array([10, 255, 255])
	# mask1 = cv.inRange(hsv, redLower, redUpper)

	# # construct a mask for the color "red", upper mask (170-180)
	# redLower = np.array([170, 50, 50])
	# redUpper = np.array([180, 255, 255])
	# mask2 = cv.inRange(hsv, redLower, redUpper)

	# construct a combined mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	mask = cv.inRange(hsv, GreenLower, GreenUpper)
	# mask = mask1 + mask2
	mask = cv.erode(mask, None, iterations = 2)
	mask = cv.dilate(mask, None, iterations = 2)

#----------------------------------OBJECT TRACKING----------------------------------
	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# # find the largest contour in the mask, then use it to
		# # compute the minimum enclosing circle and centroid
		# c = max(cnts, key = cv.contourArea)
		# ((x, y), radius) = cv.minEnclosingCircle(c)
		
		# find the largest contour in the mask, then use it to
		# compute the bounding rectangle and centroid
		c = max(cnts, key = cv.contourArea)		
		
		# use this method when tracking spheres or balls,
		# bounding box will not rotate
		# x, y, w, h = cv.boundingRect(c)
		
		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
		# # only proceed if the radius meets a minimum size
		# if radius > 10:
		# 	# draw the bounding box and centroid on the frame
		# 	cv.circle(src, (int(x), int(y)), int(radius), (0, 255, 255), 2)
		# 	# cv.rectangle(src, (x, y), (x + w, y + h), (255, 255, 255), 2)
		# 	cv.circle(src, center, 5, (0, 0, 255), -1)

		# use this method when tracking rectangular objects,
			# bounding box will rotate
		rect = cv.minAreaRect(c)
		box = cv.boxPoints(rect)
		box = np.int0(box)
		src = cv.drawContours(src, [box], 0, (0, 0, 255), 1)
		cv.circle(src, center, 5, (0, 0, 255), -1)
		print 'Bounding Box Coordinates BL - BR CW:', box
	
	print 'Result Frame Per Second:', frame_cnt / (time.time() - start_time)

	# show the frame to our screen
	cv.imshow("Frame", src)
	char = cv.waitKey(1)
	if char == 27:
		break
	frame_cnt += 1

test.Stop()  
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()
