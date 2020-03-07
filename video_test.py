import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)

while(True):
	ret, frame = cap.read()
	grey = cv.cvtColor(frame,cv.COLOR_BGR2GRAY)

	cv.imshow('frame', grey)
	if cv.waitkey(1) & 0xFF == ord('q'):
		break

cap.release
cv.destroyAllWindows()
