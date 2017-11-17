#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

TAG_SIZE = 17.7 #cm

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/io/internal_camera/head_camera/image_raw',Image,self.callback)
        self.fast = cv2.FastFeatureDetector()
        self.fast.setBool('nonmaxSuppression',0)
        self.fast.setInt('threshold', 30)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        crop = cv_image[150:rows-150,150:cols-150]
        kp = self.fast.detect(crop, None)
        print(len(kp))
        for k in kp:
            x = k.pt[0] + 150
            y = k.pt[1] + 150
            k.pt = (x,y)
        img2 = cv2.drawKeypoints(cv_image, kp, color=(255,0,0))
        cv2.imshow("Image window", img2)
        cv2.waitKey(3)

        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
