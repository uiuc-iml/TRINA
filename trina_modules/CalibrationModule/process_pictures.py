import numpy as np
import cv2
import cv2.aruco as aruco
# import cv2.aruco as aruco
from klampt.math import vectorops
from klampt.math import so3
import imutils

def process(num_pic = 18,dimy = 1920):
	"""
	This function process the pictures to find the location of the center of the markers
	"""
	center_pts = []
	for i in range(num_pic): #total of 18 pics
		# data2=open('calibration_pt_'+str(i)+'.txt','r')
		# columnPts=[]
		# for line in data2:
		# 	line=line.rstrip()
		# 	l=[num for num in line.split(' ')]
		# 	l2=[float(num) for num in l]
		# 	columnPts.append(l2)
		# data2.close()

		# ## Detect Corners
		# frame=cv2.imread('calbration_pic_'+str(i)+'.png')
		# #yellowLower = (25, 86, 6)
		# #yellowUpper = (35, 255, 255)
		# yellowLower = (20, 86, 6)    #These thresholds work better in hudson's lighting condition
		# yellowUpper = (40, 255, 255)

		# # resize the frame, blur it, and convert it to the HSV
		# # color space
		# #frame = imutils.resize(frame, width=600) # not sure if this is needed
		# blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		# hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# # construct a mask for the color "green", then perform
		# # a series of dilations and erosions to remove any small
		# # blobs left in the mask
		# mask = cv2.inRange(hsv, yellowLower, yellowUpper)
		# mask = cv2.erode(mask, None, iterations=2)
		# mask = cv2.dilate(mask, None, iterations=2)

		# # find contours in the mask and initialize the current
		# # (x, y) center of the ball
		# cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		# cv2.CHAIN_APPROX_SIMPLE)
		# cnts = imutils.grab_contours(cnts)
		# center = None

		# # only proceed if at least one contour was found
		# if len(cnts) > 0:
		# 	# find the largest contour in the mask, then use
		# 	# it to compute the minimum enclosing circle and
		# 	# centroid
		# 	c = max(cnts, key=cv2.contourArea)
		# 	((x, y), radius) = cv2.minEnclosingCircle(c)
		# 	M = cv2.moments(c)
		# 	center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

		# 	# only proceed if the radius meets a minimum size
		# 	if radius > 10:
		# 	# draw the circle and centroid on the frame,
		# 	# then update the list of tracked points
		# 		cv2.circle(frame, (int(x), int(y)), int(radius),
		# 		(0, 255, 255), 2)
		# 		cv2.circle(frame, center, 5, (0, 0, 255), -1)
		# 		p=columnPts[center[1]*dimy+center[0]] #y*dimy+x
		# 		center_pts.append(p)

		frame=cv2.imread('calbration_pic_'+str(i)+'.png')
		fx=1055.28
		fy=1054.69
		cx=982.61
		cy=555.138
		k1=-0.0426844
		k2=0.0117943
		k3=-0.00548354
		p1=0.000242741
		p2=-0.000475926
		matrix_coefficients = np.array([[fx,0,cx],\
		[0,fy,cy],\
		[0,0,1]])

		distortion_coefficients = np.array([k1,k2,p1,p2,k3])

		distortion_coefficients = distortion_coefficients.flatten()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale

		aruco_dict = aruco.Dictionary_get(aruco.DICT_ARUCO_ORIGINAL)  # Use 5x5 dictionary to find markers
		parameters = aruco.DetectorParameters_create()  # Marker detection parameters
		# lists of ids and the corners beloning to each id
		corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
		parameters=parameters)
		# cameraMatrix=matrix_coefficients,
		# distCoeff=distortion_coefficients)
		if np.all(ids is not None):  # If there are markers found by detector
			# print(ids)
			for j in range(0, len(ids)):  # Iterate in markers
				if ids[j][0] == 5:
					# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
					rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[j], 0.052, matrix_coefficients,
																				distortion_coefficients)
					(rvec - tvec).any()  # get rid of that nasty numpy value array error
					
					#aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
					#aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.01)  # Draw Axis
					marker_p = tvec[0][0]
					print('p in camera:',marker_p)
					center_pts.append(marker_p.tolist())
					break
		else:
			print('No Color Blob Detected')

		#cv2.imwrite('processed_pic_'+str(i)+'.png',frame)
		print(i)

	return center_pts

if __name__ == "__main__":
	generation()