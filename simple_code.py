import imutils
import liboCams
import cv2 as cv
import numpy as np
import math
import time
import sys
# import FUNKYTIME as FT

# Some setup
w_im = 480  # image width in pixels
h_im = 360  # image height in pixels

# Find the distance of the object relative to the camera
def FindRelativeDistance(contours):
	if cv.contourArea(contours) < 500:
		RelativeDistance = 2.021 * math.exp(-0.0021 * cv.contourArea(c)) + 0.07472 * math.exp(0.0031 * cv.contourArea(c))
	else:
		RelativeDistance = 0.9576 * math.exp(-0.001674 * cv.contourArea(c)) + 0.6411 * math.exp(-0.0001057 * cv.contourArea(c))
	return RelativeDistance

def GetCentroidData():
	x, y, w, h = GetTargetPosition()

	if x is None:  # if nothing is in frame, don't send any information
		return "No Object Detected:", None

	if (x + w == 640 or   # if target is at edge, don't send any information
			x - w <= 0 or
			y + h == 480 or
			y - h <= 0):
		return None, None, None

	# Equation 1 from Pestana "Computer vision based general object following"
	f_u = (x + (w / 2)) / w_im
	f_v = (y + (h / 2)) / h_im
	f_delta = math.sqrt((w_im * h_im) / (w * h))
	Center = [f_u, f_v]

	return Center

# Find the oCam
devpath = liboCams.FindCamera("oCam")
if devpath is None:
  exit()

test = liboCams.oCams(devpath, verbose = 1)

# Get the format list for the oCam
print "Format List"
fmtlist = test.GetFormatList()
for fmt in fmtlist:
  print "\t", fmt

# Set the format for the oCam (8-bit Bayer GRGR/BGBG,
# 640L, 480L, 60)
print "SET", 8, fmtlist[8]
test.Set(fmtlist[8])

# Get the current camera parameters and set new
# camera prameters before recording
print "Control List"
ctrlist = test.GetControlList()
for key in ctrlist:
	print "\t", key, "\tID:", ctrlist[key]
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
		print "Error opening image!"
		print "Usage: simple_code.py [oCam -- default" + default_file + "] \n"
		exit()

	# blur the frame and convert it to the HSV color space
	src = imutils.resize(src, width = 480)
	blur1 = cv.medianBlur(src, 1)
	blur2 = cv.GaussianBlur(blur1, (9, 9), 0)
	blur3 = cv.bilateralFilter(blur2, 3, 50, 50)
	hsv = cv.cvtColor(blur3, cv.COLOR_BGR2HSV)
	# gray = cv.cvtColor(blur3, cv.COLOR_BGR2GRAY)

	# Define the lower and upper boundaries of the "green"
	# ball in the HSV color space
	GreenLower = (29, 86, 6)
	GreenUpper = (64, 255, 255)

	# # Construct a mask for the color "red", Lower mask (0-10)
	# redLower = np.array([0, 50, 50])
	# redUpper = np.array([10, 255, 255])
	# mask1 = cv.inRange(hsv, redLower, redUpper)

	# # Construct a mask for the color "red", upper mask (170-180)
	# redLower = np.array([170, 50, 50])
	# redUpper = np.array([180, 255, 255])
	# mask2 = cv.inRange(hsv, redLower, redUpper)

	# Construct a combined mask for the desired color, then perform
	# a series of dilations and erosions to remove any small
	# unwanted colors left in the mask
	mask = cv.inRange(hsv, GreenLower, GreenUpper)
	# mask = mask1 + mask2
	kernel = np.ones((4, 4), np.uint8)
	# mask = cv.erode(mask, kernel, iterations = 2)
	# mask = cv.dilate(mask, kernel, iterations = 4)
	# mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)
	mask = cv.morphologyEx(mask, cv.MORPH_GRADIENT, kernel)

	# # corner detection
	# corners = cv.goodFeaturesToTrack(mask, 4, 0.1, 10)
	# corners = np.int0(corners)

	# for i in corners:
	# 	x, y = i.ravel()
	# 	cv.circle(src, (x, y), 3, 255, -1)

	# Find contours in the mask and initialize the current
	# (x, y) center of the object
	contours = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	contours = imutils.grab_contours(contours) # grabs the appropriate tuple value based on the OpenCV version
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
		# the bounding box will rotate up to 90 degrees
		points = cv.minAreaRect(approx)
		box = cv.boxPoints(points)
		box = np.int0(box)

		# # Use this method when tracking a sphere or ball,
		# # the bounding box will not rotate
		# x, y, w, h = cv.boundingRect(approx)
		# print "x:", x, "y:", y, "w:", w, "h:", h

		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# # Only proceed if the area meets a minimum size
		# if cv.contourArea(c) >= 300:
		# 	cv.drawContours(src, [approx], 0, (0, 255, 255), 1)
		# 	cv.rectangle(src, (x, y), (x + w, y + h), (0, 0, 255), 2)
		# 	cv.circle(src, center, 5, (0, 0, 255), -1)
		# 	print "Bounding Box:", (x, y), ((x + w), y), ((x + w), (y + h)), (x, (y + h))

		# else:
		# 	print "No Object Detected"

		# Only proceed if the area meets a minimum size
		if cv.contourArea(c) >= 300:
			cv.drawContours(src, [approx], 0, (0, 255, 255), 1)
			cv.drawContours(src, [box], 0, (0, 0, 255), 2)
			cv.circle(src, center, 5, (0, 0, 255), -1)
			print "Bounding Box (BL-BR CW):", box
			print "Contour Area:", cv.contourArea(c) # prints the contour area, i.e., size of the object
			print "Bounding Box Coordinates:", cv.minAreaRect(approx) # prints the bounding rectangle verteces and angle of rotation

		else:
			print "No object Detected"

		# Find the coordinates of the centroid in the image frame
		Center = GetCentroidData(x, y, w, h)
		print "Center Coordinates:", center

		# Find the distance of the object relative to the camera
		RelativeDistance = FindRelativeDistance(c)
		print "Relative Distance to Object:", RelativeDistance

	print "Result Frame Per Second:", frame_cnt / (time.time() - start_time)

	# Show the frame on our screen
	# cv.imshow("Mask", mask)
	cv.imshow("Frame", src)
	char = cv.waitKey(1) & 0xFF
	if char == 27:
		break
	frame_cnt += 1

test.Stop()
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()
