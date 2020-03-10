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

while True:
    frame = test.GetFrame()
    rgb = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
    src = rgb 
    #cd cv.imshow(test.cam.card, rgb)
    
    #----------------------------------RED BALL DETECTION----------------------------------
    # Check if image is loaded fine
    if src is None:
        print ('Error opening image!')
        print ('Usage: hough_circle.py [image_name -- default ' + default_file + '] \n')
        exit()
    ## [load]

    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-b", "--buffer", type=int, default=32,
        help="max buffer size")
    args = vars(ap.parse_args())
    
    # ## [convert_to_gray]
    # # Convert it to gray
    # gray = cv.cvtColor(src, cv.COLOR_BGR2GRAY)
    # ## [convert_to_gray]

    # ## [reduce_noise]
    # # Reduce the noise to avoid false circle detection
    # gray = cv.medianBlur(gray, 5)
    ## [reduce_noise]

    # resize the frame, blur it, and convert it to the HSV
    # color space
    src = imutils.resize(rgb, width=600)
    blur1 = cv.medianBlur(src, 5)
    blur2 = cv.GaussianBlur(blur1, (11, 11), 0)
    hsv = cv.cvtColor(blur2, cv.COLOR_BGR2HSV)
    
    # define the lower and upper boundaries of the "red"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    redLower = (0, 50, 50)
    redUpper = (10, 255, 255)
    pts = deque(maxlen=args["buffer"])

    # construct a mask for the color "red", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    mask = cv.inRange(hsv, redLower, redUpper)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)

    ## [houghcircles]
    rows = mask.shape[0]
    circles = cv.HoughCircles(mask, cv.HOUGH_GRADIENT, 1, rows / 8,
                               param1=100, param2=30,
                               minRadius=100, maxRadius=10000)
    ## [houghcircles]

    ## [draw]
    if circles is not None:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            center = (i[0], i[1])
            # circle center
            cv.circle(mask, center, 1, (0, 100, 100), 3)
            # circle outline
            radius = i[2]
            cv.circle(mask, center, radius, (255, 0, 255), 3)
    ## [draw]

    # show the frame to our screen
    cv.imshow("Frame", src)
    char = cv.waitKey(1)
    if char == 27:
        break
    frame_cnt += 1

print 'Result Frame Per Second:', frame_cnt/(time.time()-start_time)
test.Stop()  
cv.destroyAllWindows()
char = cv.waitKey(1)
test.Close()
