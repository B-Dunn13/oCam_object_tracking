import imutils
import liboCams
import cv2 as cv
import numpy as np
import math
import time
import sys
# import FUNKYTIME as FT

# Find the distance of the object relative to the camera
def FindRelativeDistance(contours):
	if cv.contourArea(contours) < 500:
		relativeDistance = 2.021 * math.exp(-0.0021 * cv.contourArea(c)) + 0.07472 * math.exp(0.0031 * cv.contourArea(c))
	else:
		relativeDistance = 0.9576 * math.exp(-0.001674 * cv.contourArea(c)) + 0.6411 * math.exp(-0.0001057 * cv.contourArea(c))
	return relativeDistance


# Find the oCam
devpath = liboCams.FindCamera('oCam')
if devpath is None:
  exit()

test = liboCams.oCams(devpath, verbose = 1)

# Get the format list for the oCam
print 'Format List'
fmtlist = test.GetFormatList()
for fmt in fmtlist:
  print '\t', fmt

# Set the format for the oCam (8-bit Bayer GRGR/BGBG,
# 640L, 480L, 60)
print 'SET', 8, fmtlist[8]
test.Set(fmtlist[8])

# Get the current camera parameters and set new
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
while True:
	frame = test.GetFrame()
	BGR = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
	src = BGR 
	
	# Check if image is loaded fine
	if src is None:
		print ('Error opening image!')
		print ('Usage: simple_code.py [oCam -- default ' + default_file + '] \n')
		exit()
	
	# Define the lower and upper boundaries of the "green"
	# ball in the HSV color space
	GreenLower = (29, 86, 6)
	GreenUpper = (64, 255, 255)

	# blur the frame and convert it to the HSV color space
	src = imutils.resize(src, width = 480)
	blur1 = cv.medianBlur(src, 1)
	blur2 = cv.GaussianBlur(blur1, (9, 9), 0)
	blur3 = cv.bilateralFilter(blur2, 3, 50, 50)
	hsv = cv.cvtColor(blur3, cv.COLOR_BGR2HSV)

	# # Construct a mask for the color "red", Lower mask (0-10)
	# redLower = np.array([0, 50, 50])
	# redUpper = np.array([10, 255, 255])
	# mask1 = cv.inRange(hsv, redLower, redUpper)

	# # Construct a mask for the color "red", upper mask (170-180)
	# redLower = np.array([170, 50, 50])
	# redUpper = np.array([180, 255, 255])
	# mask2 = cv.inRange(hsv, redLower, redUpper)

	# Construct a combined mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# unwanted colors left in the mask
	mask = cv.inRange(hsv, GreenLower, GreenUpper)
	# mask = mask1 + mask2
	kernel = np.ones((4, 4), np.uint8)
	# mask = cv.erode(mask, kernel, iterations = 2)
	# mask = cv.dilate(mask, kernel, iterations = 4)
	# mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
	mask = cv.morphologyEx(mask, cv.MORPH_GRADIENT, kernel)


	# Find contours in the mask and initialize the current
	# (x, y) center of the ball
	contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	contours = imutils.grab_contours(contours)
	center = None

	# Only proceed if at least one contour was found
	if len(contours) > 0:		
		# Find the largest contour in the mask, then use it to
		# compute the minimum area rectangle and centroid
		c = max(contours, key = cv.contourArea)
		
		# Contour approximation
		epsilon = 0.001*cv.arcLength(c, True)
		approx = cv.approxPolyDP(c, epsilon, True)

		# Use this method when tracking rectangular objects,
		# the bounding box will rotate
		points = cv.minAreaRect(approx)
		box = cv.boxPoints(points)
		box = np.int0(box)

		# Use this method when tracking a sphere or ball,
		# the bounding box will not rotate
		# x, y, w, h = cv.boundingRect(approx)

		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
		# # Only proceed if the area meets a minimum size
		# if cv.contourArea(c) > 300:
			# cv.rectangle(src, (x, y), (x + w, y + h), (255, 255, 255), 2)
			# cv.circle(src, center, 5, (0, 0, 255), -1)
			# print "Bounding Box:", (x, y), ((x + w), y), ((x + w), (y + h)), (x, (y + h))
		
		# elif cv.contourArea(c) < 300:
			# print "No object Detected"

		# Only proceed if the area meets a minimum size
		if cv.contourArea(c) > 300:
			cv.drawContours(src, [approx], 0, (0, 255, 255), 1)
			cv.drawContours(src, [box], 0, (0, 0, 255), 2)
			cv.circle(src, center, 5, (0, 0, 255), -1)
			print "Bounding Box (BL-BR CW):", box
			print "Area:", cv.contourArea(c) # prints the contour area, i.e., size of the object
			print "IDK:", cv.minAreaRect(approx) # prints the bounding rectangle verteces and angle of rotation

		elif cv.contourArea(c) < 300:
			print "No object Detected"

		# Find the distance of the object relative to the camera
		# relativeDistance = FT.FindRelativeDistance(c)
		relativeDistance = FindRelativeDistance(c)
		print "Relative Distance to Object: ", relativeDistance

	print 'Result Frame Per Second:', frame_cnt / (time.time() - start_time)

	# Show the frame on our screen
	cv.imshow("Mask", mask)
	cv.imshow("Frame", src)
	char = cv.waitKey(1) & 0xFF
	if char == 27:
		break
	frame_cnt += 1

test.Stop()  
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()