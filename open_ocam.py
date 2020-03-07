import cv2

camera = cv2.VideoCapture()
camera.open("/dev/v4l/by-id/usb-Alcor_Micro__Corp._USB_2.0_WebCamera-video-index0")
#camera.open("/dev/v4l/by-id/usb-WITHROBOT_Inc._oCam-1CGN-U-T_SN_2EA2D136-video-index0")
print(camera.isOpened())
print(camera.read())