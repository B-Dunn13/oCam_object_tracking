import asyncio
import time
import math
import cv2 as cv
from CameraUtil import Video
from mavsdk import System
from mavsdk import (OffboardError, VelocityBodyYawspeed, PositionNedYaw)
import imutils
import liboCams
import numpy as np
import sys

# -----------------------FUNCTIONS-----------------------
# Some setup
D = RelativeDistance # Depth of the point relative to the camera frame
f = 0.0036 # camera focal length (m)
FocalLength = f
w_im = 640  # image width in pixels
h_im = 480  # image height in pixels
got_initial_frame = False
initial_f_u = None
initial_f_v = None
initial_f_delta = None
psi_telem_ref = 90
psi_telem = None
theta_centroid_ref = 0
theta_centroid = None
FOV = 65
FOV_u = 52
FOV_v = 39
A_exp = 100000
d_exp = 10
PreviousRelativeDistance = 0
prev_delta_x_tme = 0
prev_delta_y_tme = 0
prev_delta_psi_tme = 0
prev_delta_z_tme = 0
x_1_star = 0
x_2_star = 640
x_3_star = 0
x_4_star = 640
y_1_star = 0
y_2_star = 0
y_3_star = 480
y_4_star = 480
center = None
x_0 = 320
y_0 = 240

kp_vx = 0.0254
kd_vx = 0.00124

kp_vy = 0.298
kd_vy = 0.145

kp_yaw = 0.990
kd_yaw = 0.119

kp_vz = 1.430
kd_vz = 0.371

euler_angles = None
drone_position = None

# Find the oCam
devpath = liboCams.FindCamera("oCam")
if devpath is None:
  exit()

test = liboCams.oCams(devpath, verbose = 1)

# Get the format list for the oCam
print "Format List"
fmtlist = test.GetFormatList()
for fmt in fmtlist:
  print '\t', fmt

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

video = test.GetFrame()
while not video.frame_available():
    continue

# Find the target in the image frame and draw a bounding box around it
def GetTargetPosition():
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

	# Define the lower and upper boundaries of the "green"
	# ball in the HSV color space
	GreenLower = (29, 86, 6)
	GreenUpper = (64, 255, 255)

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

	contours, hierarchy = cv2.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	contours = imutils.grab_contours(contours) # grabs the appropriate tuple value based on the OpenCV version

	# Only proceed if at least one contour was found
	if len(contours) > 0:
		# Find the largest contour in the mask, then use it to
		# compute the minimum area rectangle and centroid
		c = max(contours, key = cv.contourArea)

		# Contour approximation
		epsilon = 0.001*cv.arcLength(c, True)
		approx = cv.approxPolyDP(c, epsilon, True)

		# Use this method when tracking a sphere or ball,
		# the bounding box will not rotate
		x, y, w, h = cv.boundingRect(approx)

		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# Only proceed if the area meets a minimum size
		if cv.contourArea(c) >= 300:
			cv.rectangle(src, (x, y), (x + w, y + h), (255, 255, 255), 2)
			cv.circle(src, center, 5, (0, 0, 255), -1)
			cv2.imshow("green", mask)
			print "Bounding Box:", (x, y), ((x + w), y), ((x + w), (y + h)), (x, (y + h))

		else:
			print "No Object Detected"
	else:
		print "No Object Detected"

	# print(red_thresholded.item(80, 100))
	print "x_bb: %3d y_bb: %3d w_bb: %3d h_bb: %3d" % (x, y, w, h)
	if cv2.waitKey(1) & 0xFF == ord("q"):
		return x
	return x, y, w, h,

# Find the distance of the object relative to the camera
def FindRelativeDistance(contours):
	if cv.contourArea(contours) < 500:
		RelativeDistance = 2.021 * math.exp(-0.0021 * cv.contourArea(c)) + 0.07472 * math.exp(0.0031 * cv.contourArea(c))

	else:
		RelativeDistance = 0.9576 * math.exp(-0.001674 * cv.contourArea(c)) + 0.6411 * math.exp(-0.0001057 * cv.contourArea(c))
	return RelativeDistance

# -----------------------IBVS CONTROL LAW-----------------------
def IBVSController():
	x, y, w, h = GetTargetPosition()
	x_1 = x
	y_1 = y
	x_2 = x + w
	y_2 = y
	x_3 = x + w
	y_3 = y + h
	x_4 = x
	y_4 = y + h
	FeaturePoints = np.matrix([[x_1], [y_1], [x_2], [y_2], [x_3], [y_3], [x_4], [y_4]])
	DesiredFeaturePoints = np.matrix([[90], [10], [550], [10], [550], [470], [90], [470]])
	Error = FeaturePoints - DesiredFeaturePoints

	FeatureJacobian = np.matrix([[(-1 / D), 0, (x_1 / D), (x_1 * y_1), -(1 + math.pow(x_1,2)), y_1],
		[0, (-1 / D), (y_1 / D), (1 + math.pow(y_1,2)), (-x_1 * y_1), -x_1],
		[(-1 / D), 0, (x_2 / D), (x_2 * y_2), -(1 + math.pow(x_2,2)), y_2],
		[0, (-1 / D), (y_2 / D), (1 + math.pow(y_2,2)), (-x_2 * y_2), -x_2],
		[(-1 / D), 0, (x_3 / D), (x_3 * y_3), -(1 + math.pow(x_3,2)), y_3],
		[0, (-1 / D), (y_3 / D), (1 + math.pow(y_3,2)), (-x_3 * y_3), -x_3],
		[(-1 / D), 0, (x_4 / D), (x_4 * y_4), -(1 + math.pow(x_4,2)), y_4],
		[0, (-1 / D), (y_4 / D), (1 + math.pow(y_4,2)), (-x_4 * y_4), -x_4]])

	PseudoInverse = inv(FeatureJacobian.transpose * FeatureJacobian) * FeatureJacobian.transpose # Moore-Penrose pseudo-inverse

	# IBVS control output
	RelativeCameraVelocity = -1 * PseudoInverse * Error
	# need to expand RelativeCameraVelocity to feed independent values to the drone controller
	v_cx = RelativeCameraVelocity[0, 0] # Lineaar velovity of the drone
	v_cy = RelativeCameraVelocity[1, 0] # Lineaar velovity of the drone
	v_cz = RelativeCameraVelocity[2, 0] # Lineaar velovity of the drone
	w_cx = RelativeCameraVelocity[3, 0] # Angular velovity of the drone
	w_cy = RelativeCameraVelocity[4, 0] # Angular velovity of the drone
	w_cz = RelativeCameraVelocity[5, 0] # Angular velovity of the drone
	return v_cx, v_cy, v_cz, w_cx, w_cy, w_cz

def GetCentroidData():
	x_bb, y_bb, w_bb, h_bb = GetTargetPosition()

	if x_bb is None:  # if nothing is in frame, don't send any information
		return "No Object Detected:", None

	if (x_bb + w_bb == 640 or   # if the target is at the edge, don't send any information
			x_bb == 0 or
			y_bb + h_bb == 480 or
			y_bb == 0):
		return "Object Leaving Frame"

	# Equation 1 from Pestana "Computer vision based general object following"
	f_u = (x_bb + (w_bb / 2)) / w_im
	f_v = (y_bb + (h_bb / 2)) / h_im
	f_delta = math.sqrt((w_im * h_im) / (w_bb * h_bb))

	return f_u, f_v, f_delta

def DecoupleCentroidData():
	f_u, f_v, f_delta = GetCentroidData()
	global got_initial_frame, initial_f_u, initial_f_v, initial_f_delta, FOV_u, FOV_v

	if f_u is None:
		return None

	if not got_initial_frame:
		initial_f_u = 1 / 2
		initial_f_v = 1 / 2
		initial_f_delta = f_delta
		got_initial_frame = True

	# Equation 2 from Pestana "Computer vision based general object following"
	delta_f_u_psi = f_u - initial_f_u
	delta_f_u_y = delta_f_u_psi - ((psi_telem_ref - euler_angles.yaw_deg) / FOV_u)
	delta_f_v_z = (f_v - initial_f_v) - ((theta_centroid_ref - euler_angles.pitch_deg) / FOV_v)
	delta_f_delta_x = f_delta - initial_f_delta

	print "delta_f_delta_x:" + str(delta_f_delta_x)
	print "f_delta:" + str(f_delta)

	return delta_f_u_psi, delta_f_u_y, delta_f_v_z, delta_f_delta_x

def GetSetpoints():
	delta_f_u_psi, delta_f_u_y, delta_f_v_z, delta_f_delta_x = DecoupleCentroidData()
	global A_exp, d_exp, w_im, h_im, prev_delta_psi_tme, prev_delta_x_tme, prev_delta_y_tme, prev_delta_z_tme
	global kp_vx, kd_vx, kp_vy, kd_vy, kp_yaw, kd_yaw, kp_vz, kd_vz

	if delta_f_u_psi is None:
		return 0

	# Z velocity controller
	ChangeInHeight = RelativeDistance - PreviousRelativeDistance
	HeightRate = (((DesiredZ - RelativeDistance) * kp_vx) + (((RelativeDistance - PreviousRelativeDistance) / (time.time() - start_time)) * (DesiredZ - RelativeDistance) * kd_vx))
	PreviousRelativeDistance = RelativeDistance

	# Pitch velocity controller
	DesiredPitchVelocity = (((DesiredPitch - MeasuredPitch) * kp_vx) + (((MeasuredPitch - PreviousMeasuredPitch) / (time.time() - start_time)) * (DesiredPitch - MeasuredPitch) * kd_vx))
	PreviousMeasuredPitch = MeasuredPitch

	print "delta_x_tme:" + str(delta_x_tme)

	return v_xr, v_yr, yawrate, v_zr

async def Exicute():
    global euler_angles
    tracking = False  # assume that target is not in frame when initializing

    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    async for is_armed in drone.telemetry.armed():
        print("Is_armed:", is_armed)
        if not is_armed:
            print("-- Arming")
            await drone.action.arm()
        break

    print("-- Setting initial setpoint")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 90.0))
    async for drone_position in drone.telemetry.odometry():
        if (math.fabs(drone_position.position_body.x_m) < 0.2 and
                math.fabs(drone_position.position_body.y_m) < 0.2 and
                math.fabs(drone_position.position_body.z_m + 10) < 0.2):
            break

    for i in range(3, 0, -1):
        print("Switching to IBVS in " + str(i) + "..")
        await asyncio.sleep(1)

    while True:
        await asyncio.sleep(1 / 30.0)
        async for euler_angles in drone.telemetry.attitude_euler():
            break

        v_xr, v_yr, yawrate, v_zr = getsetpoints()

        print("vx: " + str(v_xr) + " vy: " + str(v_yr) + " vz: " + str(v_zr) + " yawrate: " + str(yawrate))

        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(v_xr, v_yr, v_zr, yawrate))

        # if x is None:
        #     await drone.offboard.set_velocity_body(
        #         VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        #     continue
        # await drone.offboard.set_velocity_body(
        #     VelocityBodyYawspeed(0.0, (x[0]-160)/10, 0.0, 0.0))


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.Exicute_until_complete(Exicute())
