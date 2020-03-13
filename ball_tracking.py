# import the necessary packages
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import liboCams

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

# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-v", "--video",
	help="path to the (optional) video file")
ap.add_argument("-b", "--buffer", type=int, default=32,
	help="max buffer size")
args = vars(ap.parse_args())

# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points
# GreenLower = (29, 86, 6)
# GreenUpper = (64, 255, 255)
pts = deque(maxlen=args["buffer"])

# # if a video path was not supplied, grab the reference
# # to the webcam
# if not args.get("video", False):
# 	vs = VideoStream(src=0).start()

# # otherwise, grab a reference to the video file
# else:
# 	vs = cv2.VideoCapture(args["video"])

# # allow the camera or video file to warm up
# # time.sleep(2.0)

# keep looping
while True:
	# grab the current frame
	frame = test.GetFrame()
	BGR = cv2.cvtColor(frame, cv2.COLOR_BAYER_GB2BGR)
	src = BGR 
	
	# Check if image is loaded fine
	if src is None:
		print ('Error opening image!')
		print ('Usage: simple_code.py [oCam -- default ' + default_file + '] \n')
		exit()
	
	# frame = vs.read()

	# # handle the frame from VideoCapture or VideoStream
	# frame = frame[1] if args.get("video", False) else frame
	
	# # if we are viewing a video and we did not grab a frame,
	# # then we have reached the end of the video
	# if frame is None:
	# 	break
	
	# resize the frame, blur it, and convert it to the HSV
	# color space
	src = imutils.resize(src, width=480)
	blurred = cv2.medianBlur(src, 3)
	blurred = cv2.GaussianBlur(blurred, (11, 11), 0)
	hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
	
	# construct a mask for the color "red", Lower mask (0-10)
	redLower = np.array([0, 50, 50])
	redUpper = np.array([10, 255, 255])
	mask1 = cv2.inRange(hsv, redLower, redUpper)

	# construct a mask for the color "red", upper mask (170-180)
	redLower = np.array([170, 50, 50])
	redUpper = np.array([180, 255, 255])
	mask2 = cv2.inRange(hsv, redLower, redUpper)

	# construct a mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	# mask = cv2.inRange(hsv, GreenLower, GreenUpper)
	mask = mask1 + mask2
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use
		# it to compute the minimum enclosing circle and
		# centroid
		c = max(cnts, key=cv2.contourArea)
		((x, y), radius) = cv2.minEnclosingCircle(c)
		M = cv2.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
		# only proceed if the radius meets a minimum size
		if radius > 10:
			# draw the circle and centroid on the frame,
			# then update the list of tracked points
			cv2.circle(src, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			cv2.circle(src, center, 5, (0, 0, 255), -1)
	
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
		cv2.line(src, pts[i - 1], pts[i], (0, 0, 255), thickness)
	
	# show the frame to our screen
	cv2.imshow("Frame", src)
	print 'Result Frame Per Second:', frame_cnt / (time.time() - start_time)
	key = cv2.waitKey(1) & 0xFF
	
	# if the 'q' key is pressed, stop the loop
	if key == ord("q"):
		break
	frame_cnt += 1

# # if we are not using a video file, stop the camera video stream
# if not args.get("video", False):
# 	vs.stop()

# # otherwise, release the camera
# else:
# 	vs.release()

# close all windows
cv2.destroyAllWindows()
