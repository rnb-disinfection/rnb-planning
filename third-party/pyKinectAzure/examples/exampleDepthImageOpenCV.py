import sys
sys.path.insert(1, '../pyKinectAzure/')

import numpy as np
from pyKinectAzure import pyKinectAzure, _k4a
import cv2

# Path to the module
# TODO: Modify with the path containing the k4a.dll from the Azure Kinect SDK
modulePath = r'/usr/lib/x86_64-linux-gnu/libk4a.so'
# for windows 'C:\\Program Files\\Azure Kinect SDK v1.4.1\\sdk\\windows-desktop\\amd64\\release\\bin\\k4a.dll'
# under linux please use r'/usr/lib/x86_64-linux-gnu/libk4a.so'

if __name__ == "__main__":

	# Initialize the library with the path containing the module
	pyK4A = pyKinectAzure(modulePath)

	# Open device
	pyK4A.device_open()

	# Modify camera configuration
	device_config = pyK4A.config
	device_config.color_resolution = _k4a.K4A_COLOR_RESOLUTION_1080P
	device_config.depth_mode = _k4a.K4A_DEPTH_MODE_WFOV_2X2BINNED
	print(device_config)

	# Start cameras using modified configuration
	pyK4A.device_start_cameras(device_config)

	k = 0
	while True:
		# Get capture
		pyK4A.device_get_capture()

		# Get the depth image from the capture
		depth_image_handle = pyK4A.capture_get_depth_image()

		# Check the image has been read correctly
		if depth_image_handle:

			# Read and convert the image data to numpy array:
			depth_image = pyK4A.image_convert_to_numpy(depth_image_handle)

			# Convert depth image (mm) to color, the range needs to be reduced down to the range (0,255)
			depth_color_image = cv2.applyColorMap(np.round(depth_image/30).astype(np.uint8), cv2.COLORMAP_JET)

			# Plot the image
			cv2.namedWindow('Colorized Depth Image',cv2.WINDOW_NORMAL)
			cv2.imshow('Colorized Depth Image',depth_color_image)
			k = cv2.waitKey(25)

			# Release the image
			pyK4A.image_release(depth_image_handle)

		pyK4A.capture_release()

		if k==27:    # Esc key to stop
			break

	pyK4A.device_stop_cameras()
	pyK4A.device_close()