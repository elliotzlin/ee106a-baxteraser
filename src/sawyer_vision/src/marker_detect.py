#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

TAG_SIZE = 17.7 #cm

class image_converter:

    def __init__(self):
        self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/io/internal_camera/head_camera/image_raw',Image,self.homography_callback)
        self.marker_sub = rospy.Subscriber('/io/internal_camera/head_camera/image_raw',Image,self.marker_callback)
        self.fast = cv2.FastFeatureDetector()
        self.fast.setBool('nonmaxSuppression',1)
        self.fast.setInt('threshold', 10)

        self.H = 0

        self.pub = rospy.Publisher('Bounding_Points', Float64MultiArray, queue_size=10)

    def homography_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        # Find squares (aka the AR tag)
        # TODO Crop cv_image so we don't capture whiteboard as a square
        squares = find_squares(cv_image)
        sqr = 0
        # Get valid square coordinate
        # TODO This relies on ARtag being centered in image; this is bad
        for sq in squares:
            min_x = 1000
            for point in sq:
                if point[0] < min_x:
                    min_x = point[0]
            if min_x < 500:
                continue
            else:
                sqr = sq
                break
        # Calculate homography
        spatial_points = np.array([[TAG_SIZE,TAG_SIZE], [0,TAG_SIZE], [0,0], [TAG_SIZE,0]])
        retval, mask = cv2.findHomography(spatial_points, sqr.astype(float)) # Cast as float else will throw error
        # Retval returned above is our homography transformation matrix
        self.H = retval
        cv2.drawContours(cv_image, [sqr], 0, (0, 255, 0), 3)
        #cv2.imshow("Image window", cv_image)
        #cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
        # Unsubscribe from topic because we have all we need
        self.image_sub.unregister()
        print(self.H)

    def marker_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        (rows,cols,channels) = cv_image.shape
        # Apply Gaussian blur to smooth image and reduce noise
        img2 = cv2.GaussianBlur(cv_image, (5, 5), 0)
        crop = img2[250:rows-300,250:cols-150]
        kp = self.fast.detect(crop, None)
        points = []
        for k in kp:
            x = k.pt[0] + 250
            y = k.pt[1] + 250
            k.pt = (x,y)
            points.append([int(x),int(y)])
        # Find the convex hull of points to make it work for boudningRect
        hull = cv2.convexHull(np.array(points))

        # Bound the points with a rectangle
        x,y,w,h = cv2.boundingRect(hull)
        # Top Right
        TRx = x + w
        TRy = y
        # Bottom left
        BLx = x
        BLy = y + h
        # COnverting the points to meters
        Q = np.linalg.inv(self.H)
        base = np.array([x, y, 1])
        bottom_left = np.array([BLx, BLy, 1])
        top_right = np.array([TRx, TRy, 1])

        new_base = Q.dot(base)
        new_left = Q.dot(bottom_left)
        new_right = Q.dot(top_right)

        n_w = np.linalg.norm(new_base[:2] - new_right[:2])
        n_h = np.linalg.norm(new_base[:2] - new_left[:2])


        bounding_points = Float64MultiArray
        bounding_points.data = [new_base[0],new_base[1],n_w,n_h]
        self.pub.publish(bounding_points)
        print(bounding_points.data)
        cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
        img3 = cv2.drawKeypoints(cv_image, kp, color=(255,0,0))
        cv2.imshow("Detect Marks", img3)
        cv2.waitKey(3)

# Find squares function taken from the opencv sample
def find_squares(img):
    def angle_cos(p0, p1, p2):
        d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
        return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1,d1)*np.dot(d2,d2)))
    img = cv2.GaussianBlur(img, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in range(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                _retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            contours, _hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and cv2.contourArea(cnt) > 1000 and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos(cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4]) for i in range(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    return squares

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
