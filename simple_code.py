import imutils
import liboCams
import cv2 as cv
import numpy as np
import time
import sys

# # Some setup
# w_im = 320  # image width in pixels
# h_im = 240  # image height in pixels
# got_initial_frame = False
# initial_f_u = None
# initial_f_v = None
# initial_f_delta = None
# psi_telem_ref = 90
# psi_telem = None
# theta_centroid_ref = 0
# theta_centroid = None
# FOV_u = 60
# FOV_v = 51
# A_exp = 100000
# d_exp = 10
# alpha_u = 320
# alpha_v = 240
# prev_delta_x_tme = 0
# prev_delta_y_tme = 0
# prev_delta_psi_tme = 0
# prev_delta_z_tme = 0

# # kp_vx = 0.0254
# kp_vx = 0.002
# kd_vx = 0.00124
# kp_vy = 0.298
# kd_vy = 0.145
# # kp_yaw = -0.990
# kp_yaw = 0.990
# # kd_yaw = -0.119
# kd_yaw = 0.119
# kp_vz = 1.430
# kd_vz = 0.371

# euler_angles = None
# drone_position = None

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

#----------------------------------RED BALL DETECTION----------------------------------
# define the lower and upper boundaries of the "red"
# ball in the HSV color space, then initialize the
# list of tracked points. Must start before the while
# loop for the previous centroid pts to show.
# BlueLower = ()
# BlueUpper = ()
# GreenLower = (29, 86, 6)
# GreenUpper = (64, 255, 255)

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
	blur = cv.medianBlur(src, 3)
	blur = cv.GaussianBlur(blur, (11, 11), 0)
	hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

	# construct a mask for the color "red", Lower mask (0-10)
	redLower = np.array([0, 50, 50])
	redUpper = np.array([10, 255, 255])
	mask1 = cv.inRange(hsv, redLower, redUpper)

	# construct a mask for the color "red", upper mask (170-180)
	redLower = np.array([170, 50, 50])
	redUpper = np.array([180, 255, 255])
	mask2 = cv.inRange(hsv, redLower, redUpper)

	# construct a combined mask for the color "red", then perform
	# a series of dilations and erosions to remove any small
	# blobs left in the mask
	# mask = cv.inRange(hsv, GreenLower, GreenUpper)
	mask = mask1 + mask2
	mask = cv.erode(mask, None, iterations = 2)
	mask = cv.dilate(mask, None, iterations = 2)

	# find contours in the mask and initialize the current
	# (x, y) center of the ball
	cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	# contours = cv.drawContours(mask, [cnts], 0, (0, 255, 0), 3)
	center = None

	# only proceed if at least one contour was found
	if len(cnts) > 0:
		# find the largest contour in the mask, then use it to
		# compute the minimum enclosing circle and centroid
		# c = max(cnts, key = cv.contourArea)
		# ((x, y), radius) = cv.minEnclosing Circle(c)
		
		# find the largest contour in the mask, then use it to
		# compute the bounding rectangle and centroid
		c = max(cnts, key = cv.contourArea)		
		
		# use this method when tracking spheres or balls,
		# bounding box will not rotate
		# x, y, w, h = cv.boundingRect(c)
		
		M = cv.moments(c)
		center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
		
		# only proceed if the radius meets a minimum size
		# if radius > 10:
			# draw the bounding box and centroid on the frame
			# cv.circle(src, (int(x), int(y)), int(radius), (0, 255, 255), 2)
			# cv.rectangle(src, (x, y), (x + w, y + h), (255, 255, 255), 2)
			# cv.circle(src, center, 5, (0, 0, 255), -1)
		
		# use this method when tracking rectangular objects,
		# bounding box will rotate
		rect = cv.minAreaRect(c)
		box = cv.boxPoints(rect)
		box = np.int0(box)
		src = cv.drawContours(src, [box], 0, (0, 0, 255), 2)
		cv.circle(src, center, 5, (0, 0, 255), -1)
		print 'Bounding Box Coordinates:', box

# def gettargetposition():
# 	frame = test.GetFrame()
# 	BGR = cv.cvtColor(frame, cv.COLOR_BAYER_GB2BGR)
# 	src = BGR 
	
# 	# Check if image is loaded fine
# 	if src is None:
# 		print ('Error opening image!')
# 		print ('Usage: simple_code.py [oCam -- default ' + default_file + '] \n')
# 		exit()
	
# 	# resize the frame, blur it, and convert it to the HSV color space
# 	# src = imutils.resize(BGR, width = 600)
# 	blur = cv.medianBlur(src, 5)
# 	blur = cv.GaussianBlur(blur, (11, 11), 0)
# 	hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)

# 	# construct a mask for the color "red", Lower mask (0-10)
# 	redLower = np.array([0, 50, 50])
# 	redUpper = np.array([10, 255, 255])
# 	mask1 = cv.inRange(hsv, redLower, redUpper)

# 	# construct a mask for the color "red", upper mask (170-180)
# 	redLower = np.array([170, 50, 50])
# 	redUpper = np.array([180, 255, 255])
# 	mask2 = cv.inRange(hsv, redLower, redUpper)

# 	# construct a combined mask for the color "red", then perform
# 	# a series of dilations and erosions to remove any small
# 	# blobs left in the mask
# 	# mask = cv.inRange(hsv, redLower, redUpper)
# 	mask = mask1 + mask2
# 	mask = cv.erode(mask, None, iterations = 2)
# 	mask = cv.dilate(mask, None, iterations = 2)

# 	contours, hierarchy = cv.findContours(mask, 1, 2)
# 	if len(contours) == 0:
# 		return None, None, None, None
# 	cnt = contours[0]
# 	x, y, w, h = cv.boundingRect(cnt)
# 	cv.rectangle(src, (x, y), (x + w, y + h), (255, 255, 255))

# 	cv.imshow('frame', src)

# 	# print(mask.item(80, 100))
# 	print("x_bb: %3d y_bb: %3d w_bb: %3d h_bb: %3d" % (x, y, w, h))
# 	if cv.waitKey(1) & 0xFF == ord('q'):
# 		return x
# 	return x, y, w, h

# def getcentroiddata():
# 	x_bb, y_bb, w_bb, h_bb = gettargetposition()

# 	if x_bb is None:  # if nothing is in frame, don't send any information
# 		return None, None, None

# 	if (x_bb + w_bb == 320 or   # if target is at edge, don't send any information
# 			x_bb - w_bb <= 0 or
# 			y_bb + h_bb == 240 or
# 			y_bb - h_bb <= 0):
# 		return None, None, None

# 	# Equation 1 from Pestana "Computer vision based general object following"
# 	f_u = (x_bb + (w_bb / 2)) / w_im
# 	f_v = (y_bb + (h_bb / 2)) / h_im
# 	f_delta = math.sqrt((w_im * h_im) / (w_bb * h_bb))

# 	return f_u, f_v, f_delta

# def decouplecentroiddata():
# 	f_u, f_v, f_delta = getcentroiddata()
# 	global got_initial_frame, initial_f_u, initial_f_v, initial_f_delta, FOV_u, FOV_v

# 	if f_u is None:
# 		return None, None, None, None

# 	if not got_initial_frame:
# 		initial_f_u = 1 / 2  # f_u
# 		initial_f_v = 1 / 2  # f_v
# 		initial_f_delta = f_delta
# 		got_initial_frame = True

# 	# Equation 2 from Pestana "Computer vision based general object following"
# 	delta_f_u_psi = f_u - initial_f_u
# 	delta_f_u_y = delta_f_u_psi - ((psi_telem_ref - euler_angles.yaw_deg) / FOV_u)
# 	delta_f_v_z = (f_v - initial_f_v) - ((theta_centroid_ref - euler_angles.pitch_deg) / FOV_v)
# 	delta_f_delta_x = f_delta - initial_f_delta

# 	print("delta_f_delta_x: " + str(delta_f_delta_x))
# 	print("f_delta: " + str(f_delta))

# 	return delta_f_u_psi, delta_f_u_y, delta_f_v_z, delta_f_delta_x


# def getsetpoints():
# 	delta_f_u_psi, delta_f_u_y, delta_f_v_z, delta_f_delta_x = decouplecentroiddata()
# 	global A_exp, d_exp, w_im, h_im, alpha_u, alpha_v, prev_delta_psi_tme, prev_delta_x_tme, prev_delta_y_tme, prev_delta_z_tme
# 	global kp_vx, kd_vx, kp_vy, kd_vy, kp_yaw, kd_yaw, kp_vz, kd_vz

# 	if delta_f_u_psi is None:
# 		return 0, 0, 0, 0

# 	# x velocity controller
# 	# delta_x_tme = delta_f_delta_x * math.sqrt(A_exp) * math.sqrt((alpha_u * alpha_v) / (w_im * h_im))
# 	delta_x_tme = delta_f_delta_x * math.sqrt(A_exp)
# 	v_xr = delta_x_tme * kp_vx  # + ((delta_x_tme - prev_delta_x_tme) / (1 / 30)) * kd_vx
# 	prev_delta_x_tme = delta_x_tme

# 	# y velocity controller
# 	delta_y_tme = delta_f_u_y * d_exp * (w_im / alpha_u)
# 	v_yr = delta_y_tme * kp_vy  # + ((delta_y_tme - prev_delta_y_tme) / (1/30)) * kd_vy
# 	prev_delta_y_tme = delta_y_tme

# 	# yawrate controller
# 	delta_psi_tme = delta_f_u_psi * FOV_u
# 	yawrate = delta_psi_tme * kp_yaw + ((delta_psi_tme - prev_delta_psi_tme) / (1 / 30)) * kd_yaw
# 	prev_delta_psi_tme = delta_psi_tme

# 	# z velocity controller
# 	delta_z_tme = delta_f_v_z * d_exp * (h_im / alpha_v)
# 	v_zr = delta_z_tme * kp_vz  # + ((delta_z_tme - prev_delta_z_tme) / (1/30)) * kd_vz
# 	prev_delta_z_tme = delta_z_tme

# 	print("delta_x_tme: " + str(delta_x_tme))

# 	return v_xr, v_yr, yawrate, v_zr

# async def run():
#     global euler_angles
#     tracking = False  # assume that target is not in frame when initializing

#     drone = System()
#     await drone.connect(system_address="udp://:14540")

#     print("Waiting for drone to connect...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print(f"Drone discovered with UUID: {state.uuid}")
#             break

#     async for is_armed in drone.telemetry.armed():
#         print("Is_armed:", is_armed)
#         if not is_armed:
#             print("-- Arming")
#             await drone.action.arm()
#         break

#     print("-- Setting initial setpoint")
#     await drone.offboard.set_velocity_body(
#         VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

#     print("-- Starting offboard")
#     try:
#         await drone.offboard.start()
#     except OffboardError as error:
#         print(f"Starting offboard mode failed with error code: \
#               {error._result.result}")
#         print("-- Disarming")
#         await drone.action.disarm()
#         return

#     await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -10.0, 90.0))
#     async for drone_position in drone.telemetry.odometry():
#         if (math.fabs(drone_position.position_body.x_m) < 0.2 and
#                 math.fabs(drone_position.position_body.y_m) < 0.2 and
#                 math.fabs(drone_position.position_body.z_m + 10) < 0.2):
#             break

#     for i in range(3, 0, -1):
#         print("Switching to IBVS in " + str(i) + "..")
#         await asyncio.sleep(1)

#     while True:
#         await asyncio.sleep(1 / 30.0)
#         async for euler_angles in drone.telemetry.attitude_euler():
#             break

#         v_xr, v_yr, yawrate, v_zr = getsetpoints()

#         print("vx: " + str(v_xr) + " vy: " + str(v_yr) + " vz: " + str(v_zr) + " yawrate: " + str(yawrate))

#         await drone.offboard.set_velocity_body(
#             # VelocityBodyYawspeed(v_xr, v_yr, v_zr, yawrate)
#             VelocityBodyYawspeed(v_xr, v_yr, v_zr, yawrate)
#         )

#         # if x is None:
#         #     await drone.offboard.set_velocity_body(
#         #         VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
#         #     continue
#         # await drone.offboard.set_velocity_body(
#         #     VelocityBodyYawspeed(0.0, (x[0]-160)/10, 0.0, 0.0))

	print 'Result Frame Per Second:', frame_cnt / (time.time() - start_time)
	# print 'Result Frame Per Second:', box
	# print 'Bounding Box Points:', 'Top Left X:', x, 'Top Left Y:', y,\
	# 'Bottom Right X:', x + w, 'Bottom Right Y:', y + h

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
