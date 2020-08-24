import numpy as np
import cv2
import cv2.aruco as aruco
from klampt.math import vectorops
from klampt.math import so3
import imutils

def process(num_pic = 18):
	"""
	This function process the pictures to find the location of the center of the markers
	"""
	center_pts = []
	for i in range(num_pic): #total of 18 pics
		data2=open('calibration_pt_'+str(i)+'.txt','r')
		columnPts=[]
		for line in data2:
			line=line.rstrip()
			l=[num for num in line.split(' ')]
			l2=[float(num) for num in l]
			columnPts.append(l2)
		data2.close()

		## Detect Corners
		frame=cv2.imread('calbration_pic_'+str(i)+'.png')
		#yellowLower = (25, 86, 6)
		#yellowUpper = (35, 255, 255)
		yellowLower = (20, 86, 6)    #These thresholds work better in hudson's lighting condition
		yellowUpper = (40, 255, 255)

		# resize the frame, blur it, and convert it to the HSV
		# color space
		#frame = imutils.resize(frame, width=600) # not sure if this is needed
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

		# construct a mask for the color "green", then perform
		# a series of dilations and erosions to remove any small
		# blobs left in the mask
		mask = cv2.inRange(hsv, yellowLower, yellowUpper)
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
				cv2.circle(frame, (int(x), int(y)), int(radius),
				(0, 255, 255), 2)
				cv2.circle(frame, center, 5, (0, 0, 255), -1)
				p=columnPts[center[1]*640+center[0]] #y*640+x
				center_pts.append(p)

		else:
			print 'No Color Blob Detected'

		# cv2.imshow('color',frame)
		# cv2.waitKey(1000)
		# cv2.destroyAllWindows()
		cv2.imwrite('processed_pic_'+str(i)+'.png',frame)
		print i

	return center_pts

if __name__ == "__main__":
	generation()