#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class line_follower():
    
    def __init__(self):
        #to give commands in moving the robot
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #to get images from the robot
        self.sub = rospy.Subscriber("/robot/camera/image_raw", Image, self.camera_callback)

        self.prev_error = 0
        self.bridge = CvBridge()
        


    def camera_callback(self,data): 
        movement = Twist() 

        try:
            camera_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        #converts image to HSV colorspace and then masks according to max and min threshold
        image_HSV = cv2.cvtColor(camera_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(image_HSV, (0,0,0), (20,0,30))
        

        # Returns the moment of the binary image
        moment = cv2.moments(mask) 
        
        if moment["m00"] == 0:
            movement.angular.z = 0.8
            print("no line detected")

        else:
            # calculate x,y coordinate of center
            cX = int(moment["m10"] / moment["m00"])
            cY = int(moment["m01"] / moment["m00"])

            h,width,c = camera_image.shape
            mid = width/2

            #pid function here
            error = mid - cX

            P = 1.5/100 * error
            D = 0/100 * (error - self.prev_error)
            
            G = P + D

            movement.angular.z = G
            movement.linear.x = 0.5

            self.prev_error = error

        self.pub.publish(movement)

def main():
    lf = line_follower()
    rospy.init_node('line_follower', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

