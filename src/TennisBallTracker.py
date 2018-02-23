#!/usr/bin/env python
# TennisBallTracker.py

import cv2
import numpy as np
import os
import rospy
from rover.msg import Ball

###################################################################################################
def main():

    pub = rospy.Publisher('ball', Ball)
    rospy.init_node('Ball')

    capWebcam = cv2.VideoCapture(0)                     # declare a VideoCapture object and associate to webcam, 0 => use 1st webcam

    if capWebcam.isOpened() == False:                           # check if VideoCapture object was associated to webcam successfully
        print "error: capWebcam not accessed successfully\n\n"          # if not, print error message to std out
        os.system("pause")                                              # pause until user presses a key so user can see error message
        return                                                          # and exit function (which exits program)

    while cv2.waitKey(1) != 27 and capWebcam.isOpened():                # until the Esc key is pressed or webcam connection is lost    
        blnFrameReadSuccessfully, imgOriginal = capWebcam.read()            # read next frame

        if not blnFrameReadSuccessfully or imgOriginal is None:             # if frame was not read successfully
            print "error: frame not read from webcam\n"                     # print error message to std out
            os.system("pause")                                              # pause until user presses a key so user can see error message
            break                                                           # exit while loop (which exits program)

        imgHSV = cv2.cvtColor(imgOriginal, cv2.COLOR_BGR2HSV)

        imgThresh = cv2.inRange(imgHSV, (35, 90, 90), (50, 255, 255))
 

        imgThresh = cv2.dilate(imgThresh, np.ones((7,7),np.uint8))        # close image (dilate, then erode)
        imgThresh = cv2.erode(imgThresh, np.ones((7,7),np.uint8))         # closing "closes" (i.e. fills in) foreground gaps

        imgThresh = cv2.GaussianBlur(imgThresh, (5, 5), sigmaX=5,sigmaY=5)                 # blur

        _,contours, _ = cv2.findContours(imgThresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0:
            cnt0 = contours[0]
            for cnt in contours:
                if (len(cnt) > len(cnt0)):
                    cnt0 = cnt

            hull = cv2.convexHull(cnt0)

            if len(hull) > 10:
                (x,y),radius = cv2.minEnclosingCircle(hull)
                center = (int(x),int(y))
                radius = int(radius)
                cv2.circle(imgOriginal,center,radius,(0,255,0),2)
                if not rospy.is_shutdown():
                    pub.publish(int(x),int(y),radius)
                #print "there is a ball at "+str(center)+" with radius " + str(radius)



        cv2.namedWindow("imgOriginal", cv2.WINDOW_AUTOSIZE)            # create windows, use WINDOW_AUTOSIZE for a fixed window size
        #cv2.namedWindow("imgThresh", cv2.WINDOW_AUTOSIZE)           # or use WINDOW_NORMAL to allow window resizing

        cv2.imshow("imgOriginal", imgOriginal)                 # show windows
        #cv2.imshow("imgThresh", imgThresh)
    # end while

    cv2.destroyAllWindows()                     # remove windows from memory

    return

###################################################################################################
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException: pass
