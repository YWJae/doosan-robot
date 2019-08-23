#!/usr/bin/env python

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
from ur5_vision_system.msg import Tracker
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
tracker = Tracker()


class ur5_vision:
    def __init__(self):
        rospy.init_node("ur5_vision", anonymous=False)
        self.track_flag = False
        self.target_found = False                # flag initialized to False
        self.cx = 400.0
        self.cy = 400.0
	self.cz = 0
        self.target_d = 0                        # d is distance camera(0) to target(+) from depth image
        self.bridge = cv_bridge.CvBridge()
        self.image_color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback, queue_size=1)
        self.image_depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy', Tracker, queue_size=1)

    # This callback function handles processing Camera color image, looking for the center of 
    # the red target.
    def image_callback(self,msg):
        # BEGIN BRIDGE
	e1 = cv2.getTickCount()
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        # BEGIN HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # END HSV
        # BEGIN FILTER
        lower_red = np.array([ 0,  100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        # find all of the contours in the mask image
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	
         
	## Loop through all of the contours, and get their areas
        area = [0.0]*len(cnts)
	self.contourLength  = len(cnts)
        for i in range(self.contourLength):
            area[i] = cv2.contourArea(cnts[i])

         #### Target #### the largest "red" object
	if self.contourLength < 1: 
		#print "No object Detected"
		pass
 	else:
		self.target_image = cnts[area.index(max(area))]
		self.area_max = cv2.contourArea(self.target_image)
                print "++++++++++++++++++++++"
                print self.area_max
		self.h, self.w, self.d = image.shape
		M = cv2.moments(self.target_image)
		
		try: self.cx = int(M['m10']/M['m00'])
		except ZeroDivisionError: self.cx = int(M['m10']/0.001)
		try: self.cy = int(M['m01']/M['m00'])
		except ZeroDivisionError: self.cy = int(M['m01']/0.001)
			

		cv2.circle(image, (self.cx, self.cy), 10, (0,0,0), -1)
		cv2.putText(image, "({}, {})".format(int(self.cx), int(self.cy)), (int(self.cx-5), int(self.cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
		cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
		
	#=====================================================================================
		if self.area_max > 1000:
		            self.track_flag = True

		else:
		            self.track_flag = False

#===========================================================================================
        e2 = cv2.getTickCount()
	time = (e2 - e1)/cv2.getTickFrequency()
        #print "time for detect = ", time 
        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(1)

    # This callback function handles processing Camera depth image, looking for the depth value 
    #   at the location of the center of the red target.	
    def depth_callback(self, msg):

        # process only if target is found
       # if self.target_found == True:

        if self.track_flag == True:

          # create OpenCV depth image using default passthrough encoding
          try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
          except CvBridgeError as e:
            print(e)

         # using target (v, u) location, find depth value of point and divide by 1000
         # to change millimeters into meters (for Camera sensors only)
          self.target_d = depth_image[self.cy, self.cx] / 1000.0 - 0.055

         # if depth value is zero, use the last non-zero depth value
          if self.target_d == 0:
            self.target_d = self.last_d
          else:
            self.last_d = self.target_d

         # record target location and publish target pose message
          self.error_x = self.cx - self.w/2
	  self.error_y = self.cy - (self.h/2+195)
	  tracker.x = self.cx
	  tracker.y = self.cy
	  tracker.z = self.target_d
	  tracker.flag1 = self.track_flag
	  tracker.error_x = self.error_x
	  tracker.error_y = self.error_y

        else:
          tracker.flag1 = self.track_flag
	  tracker.x = self.cx
	  tracker.y = self.cy
	  tracker.z = 0.0
	  tracker.error_x = 0
	  tracker.error_y = 0

        self.cxy_pub.publish(tracker)




if __name__ == '__main__':

   # start up the detector node and run until shutdown by interrupt
   try:
	follower=ur5_vision()
	rospy.spin()

   except rospy.ROSInterruptException:
        rospy.loginfo("Detector node terminated.")

   # close all terminal windows when process is shut down
   cv2.destroyAllWindows()

