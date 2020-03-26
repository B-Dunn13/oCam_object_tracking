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

# -----------------------QUADROTOR MODEL DYNAMICS-----------------------
# Variables and parameters
# Euler angles:
psi = ? # yaw angle from drone
phi = ? # roll angle from drone
theta = ? # pitch angle from drone
f = 0.0036 # camera focal length (m)
FocalLength = f
d = 0.225 # distance from motor to center of mass (m)
mu = ? # constant relating the torque to the rotation speed of the motors
m = 1.7 # mass of the drone (kg)
g = 9.81 # acceleration due to gravity (m/s^2)
T = T1 + T2 + T3 +T4 # total thrust from each motor
TotalThrust = [0:1] # thrust (%)

# Inertial frame (I):
x_I
y_I
z_I

v_c
v_ct

I_xx = ? # moment of inertia along the x-axis
I_yy = ? # moment of inertia along the y-axis
I_zz = ? # moment of inertia along the z-axis

# Nonlinearized model:
x_ddotI = (T / m) * ((math.sin(phi) * math.sin(psi)) + (math.cos(phi) * math.cos(psi) * math.sin(theta)))
y_ddotI = (T / m) * ((math.cos(phi) * math.sin(theta) * math.sin(psi)) - (math.sin(phi) * math.cos(psi)))
z_ddotI = ((T / m) * (math.cos(theta) * math.cos(phi))) - g

# Body fixed frame (B):
x_B
y_B
z_B

# F = ma = m*v_dot + m*omega x v
F = (m * g * z_I) - (T * z_B)
CombinedThrust = np.matrix([[0], [0], [T]])
GravitationalForce = np.matrix([[0], [0], [-m * g]])
LinearVelocity = np.matrix([[u], [v], [w]])
LinearAcceleration = np.matrix([[u_dot], [v_dot], [w_dot]])
AngularVelocity = np.matrix([[p], [q], [r]])
AngularAcceleration = np.matrix([[p_dot], [q_dot], [r_dot]])
# OrthogonalVelocity = np.matrix([[(q * w) - (v * r)], [(u * r) - (p * w)], [(p * v) - (u * q)]])

# Converting from Inertial to Body frame:
I2B_RotationMatrix = np.matrix([[(math.cos(psi) * math.cos(theta)), (math.sin(psi) * math.cos(theta)), (-math.sin(theta))],
	[(math.cos(psi) * math.sin(theta) * math.sin(phi)) - (math.sin(psi) * math.cos(phi)), (math.sin(psi) * math.sin(theta) * math.sin(phi)) + (math.cos(psi) * math.cos(phi)), (math.cos(theta) * math.sin(phi))],
	[(math.cos(psi) * math.sin(theta) * math.cos(phi)) + (math.sin(psi) * math.sin(phi)), (math.sin(psi) * math.sin(theta) * math.cos(phi)) - (math.cos(psi) * math.sin(phi)), (math.cos(theta) * math.cos(phi))]])

TotalForce = CombinedThrust + (I2B_RotationMatrix * GravitationalForce)
TotalForce = (m * LinearAcceleration) + (m * np.cross(AngularVelocity, LinearVelocity))

u_dot = (g * math.sin(theta)) + (v * r) - (q * w)
v_dot = (-g * math.cos(theta) * math.sin(phi)) + (p * w) - (u * r)
w_dot = (T / m) - (g * math.cos(theta) * math.cos(phi)) + (w * q) - (p *v)

M = np.matrix([[M_x], [M_y], [M_z]])
Inertia = np.matrix([[I_xx, 0, 0], [0, I_yy, 0], [0, 0, I_zz]])
BodyMoment = (Inertia * AngularAcceleration) + np.cross(AngularVelocity, (Inertia * AngularVelocity))

# Equation 2.2: angular acceleration
p_dot = (M_x - ((I_zz - I_yy) * (q *r))) / I_xx
q_dot = (M_y - ((I_xx - I_zz) * (p *r))) / I_yy
r_dot = M_z / I_zz

# -----------------------LINEARIZATION APPROXIMATION-----------------------
# Roll annd pitch are assumed to be small, sin(a) = a and cos(a) = 1
# Equation 2.3: linerized model, small roll and pitch angles
x_ddot = (T / m) * ((phi * math.sin(psi)) + (theta * math.cos(psi)))
y_ddot = (T / m) * ((theta * math.sin(psi)) - (phi * math.cos(psi)))
z_ddot = (T / m) - g

# Equation 2.6: linearized angular accelerations
phi_ddot = (M_x - (I_zz - I_yy) * (theta_dot * psi_dot)) / I_xx
theta_ddot = (M_y - (I_xx - I_zz) * (phi_dot * psi_dot)) / I_yy
psi_ddot = M_z / I_zz

# Equation 2.7: combined thrust input
K = np.matrix([[1, 1, 1, 1], [0, -d, 0, d], [-d, 0, d, 0], [-mu, mu, -mu, mu]])
MotorThrust = np.matrix([[T_1], [T_2], [T_3], [T_4]])
ThrustInput = K * MotorThrust
U = np.matrix([[U_1], [U_2], [U_3], [U_4]])

# Equation 2.9: quadrotor controller input
U_1 = ((T / m) * (math.cos(theta) * math.cos(phi))) - g
U_2 = (M_x - ((I_zz - I_yy) * (theta_dot * psi_dot))) / I_xx
U_3 = (M_y - ((I_xx - I_zz) * (phi_dot * psi_dot))) / I_yy
U_4 = M_z / I_zz

# Camera frame (C):
P_c = (X_c, Y_c, Z_c)

# Image frame (P):
p_0 = np.matrix([x_0, y_0])
p = np.matrix([x, y])
# Equation 2.1: principal point relationship
x = (f * X_c / Z_c) + x_0
y = (f * Y_c / Z_c) + y_0 
P_p = (x - x_0, y - y_0)

# Target frame (T):
v_t = 0 # target velocity

# Virtual camera frame (V):


# -----------------------IBVS CONTROL LAW-----------------------
s = np.matrix([[x_1], [y_1], [x_2], [y_2], [x_3], [y_3], [x_4], [y_4]])

# Equation 3.6: derivative of the projection equations (2.1)
x_dot = (f * X_c_dot / Z_c) - (f * X_c * Z_c_dot / math.pow(Z_c, 2))
y_dot = (f * Y_c_dot / Z_c) - (f * Y_c * Z_c_dot / math.pow(Z_c, 2))

InteractionMatrix = np.matrix([[(-1 / Z), 0, (x / Z), (x * y), -(1 + math.pow(x,2)), y],
	[0, (-1 / Z), (y / Z), (1 + math.pow(y,2)), (-x * y), -x]]) # Z is the depth of the point relative to the camera frame

FeatureJacobian = np.matrix([[(-1 / Z), 0, (x_1 / Z), (x_1 * y_1), -(1 + math.pow(x_1,2)), y_1],
	[0, (-1 / Z), (y_1 / Z), (1 + math.pow(y_1,2)), (-x_1 * y_1), -x_1], 
	[(-1 / Z), 0, (x_2 / Z), (x_2 * y_2), -(1 + math.pow(x_2,2)), y_2],
	[0, (-1 / Z), (y_2 / Z), (1 + math.pow(y_2,2)), (-x_2 * y_2), -x_2],
	[(-1 / Z), 0, (x_3 / Z), (x_3 * y_3), -(1 + math.pow(x_3,2)), y_3],
	[0, (-1 / Z), (y_3 / Z), (1 + math.pow(y_3,2)), (-x_3 * y_3), -x_3],
	[(-1 / Z), 0, (x_4 / Z), (x_4 * y_4), -(1 + math.pow(x_4,2)), y_4],
	[0, (-1 / Z), (y_4 / Z), (1 + math.pow(y_4,2)), (-x_4 * y_4), -x_4]])

CameraVelocity = np.matrix([[v_cx], [v_cy], [v_cz]])

# Equation 3.8: camera velocity relative to target velocity
RelativeCameraVelocity = np.matrix([[v_cx - v_tx], [v_cy - v_ty], [v_cz - v_tz], [w_cx], [w_cy], [w_cz]])

# Equation 3.9: image kinematics
s_dot =  FeatureJacobian * RelativeCameraVelocity

FeaturePoints = np.matrix([x_1, y_1, x_2, y_2, x_3, y_3, x_4, y_4])
DesiredFeaturePoints = np.matrix([x_1_star, y_1_star, x_2_star, y_2_star, x_3_star, y_3_star, x_4_star, y_4_star])
Error = FeaturePoints - DesiredFeaturePoints

# Equation 3.10: derivative of the error
e_dot = s_dot

# Equation 3.11: camera velocity output
# v_c = -lambda * PseudoInverse * Error

# -----------------------FUNCTIONS-----------------------
# Some setup
w_im = 480  # image width in pixels
h_im = 360  # image height in pixels
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
prev_delta_x_tme = 0
prev_delta_y_tme = 0
prev_delta_psi_tme = 0
prev_delta_z_tme = 0
x_1_star = 0
x_2_star = 480
x_3_star = 0
x_4_star = 480
y_1_star = 0
y_2_star = 0
y_3_star = 360
y_4_star = 360
center = None
x_0 = 240
y_0 = 180

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

# Find the target in the image frame
# and draw a bounding box around it
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
	return x, y, w, h, (x, y), ((x + w), y), ((x + w), (y + h)), (x, (y + h))

# Find the distance of the object relative to the camera
def FindRelativeDistance(contours):
	if cv.contourArea(contours) < 500:
		RelativeDistance = 2.021 * math.exp(-0.0021 * cv.contourArea(c)) + 0.07472 * math.exp(0.0031 * cv.contourArea(c))
	
	else:
		RelativeDistance = 0.9576 * math.exp(-0.001674 * cv.contourArea(c)) + 0.6411 * math.exp(-0.0001057 * cv.contourArea(c))
	return RelativeDistance

def GetCentroidData():
	x_bb, y_bb, w_bb, h_bb = GetTargetPosition()

	if x_bb is None:  # if nothing is in frame, don't send any information
		return "No Object Detected:", None

	if (x_bb + w_bb == 640 or   # if the target is at the edge, don't send any information
			x_bb <= 0 or
			y_bb + h_bb == 480 or
			y_bb <= 0):
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

	# x velocity controller
	ChangeInRelativeDistance = RelativeDistance - PreviousRelativeDistance
	PreviousRelativeDistance = RelativeDistance
	DesiredPitch = (((Desiredx - Measuredx) * kp_vx) + (((Measuredx - PreviousMeasuredx) / (time.time() - start_time)) * (Desiredx - Measuredx) * kd_vx))
	PreviousMeasuredx = Measuredx

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
