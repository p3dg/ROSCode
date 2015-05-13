#!/usr/bin/env python
import roslib
roslib.load_manifest('hsv_filter')
import sys
import rospy
import cv
import time
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class hsv_node:

    def __init__(self):
        rospy.init_node('hsv_node')

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("/static_image", Image, self.handle_image)
        self.image_pub = rospy.Publisher('/output_image', Image)

        self.flags = cv.CV_FLOODFILL_FIXED_RANGE | cv.CV_FLOODFILL_MASK_ONLY | (255 << 8)
        self.debug = True

    def handle_image(self, data):

        try:
            """ Convert the raw image to OpenCV format """
            src = self.bridge.imgmsg_to_cv(data, "bgr8")
            self.process_image(src)
        except CvBridgeError, e:
            print e
            return

    def process_image(self, img):
            self.orig_size = (width, height) = cv.GetSize(img)
            FACTOR = 6
            self.small_size = (self.orig_size[0]/FACTOR,self.orig_size[1]/FACTOR)
            self.small_size_mask = (self.small_size[0] + 2,self.small_size[1] + 2)

            dst = cv.CreateImage(self.small_size, cv.IPL_DEPTH_8U, 3)
            hsv = cv.CreateImage(self.small_size, cv.IPL_DEPTH_8U, 3)
            sm = cv.CreateImage(self.small_size, cv.IPL_DEPTH_8U, 3);
            
            out = cv.CreateImage(self.orig_size, cv.IPL_DEPTH_8U, 1)
            fill = cv.CreateImage(self.small_size_mask, cv.IPL_DEPTH_8U, 1)

            h_plane = cv.CreateImage(cv.GetSize(sm), cv.IPL_DEPTH_8U, 1);

            cv.Resize(img, sm)

            #TODO: Tweak these parameters
            # Dilation may be necessary
            cv.Dilate(sm,sm, iterations = 2)
            cv.PyrMeanShiftFiltering(sm, dst, 15, 30)

            cv.CvtColor(dst, hsv, cv.CV_BGR2HSV);

            (w,h) = self.small_size
            point = (w/2, h-h/6)

            cv.Split(hsv, h_plane, None, None, None)

            UPPER = 90
            LOWER = 40
            RANGE = 4

            # To be more probablistic, we could calcualte a histogram of a safe region
            # and treshhold on the most largest and smallest H-values that appear most frequently.
            # But this is likely good enough.
            # cv.Threshold(h_plane, h_plane, UPPER, 105, cv.CV_THRESH_BINARY_INV)
            # cv.Threshold(h_plane, h_plane, LOWER, 165, cv.CV_THRESH_TOZERO)
            # A gaussian blur may also improve results
            cv.Zero(fill)
            cv.FloodFill(h_plane, point, cv.RGB(255, 255, 255), (RANGE,)*4, (RANGE,)*4, self.flags, fill)
            
            #publish Image
            cv.Resize(fill, out)
            image_message = self.bridge.cv_to_imgmsg(out, encoding="passthrough")
            self.image_pub.publish(image_message)
            
            # if(self.debug):
            #     cv.NamedWindow("H", 0)
            #     cv.ShowImage("H", h_plane)
            #     
            #     cv.NamedWindow("HSV", 0)
            #     cv.ShowImage("HSV", hsv)
            # 
            #     cv.NamedWindow("Mean Shift", 0)
            #     cv.ShowImage("Mean Shift", dst)
            # 
            #     cv.NamedWindow("Fill", 0)
            #     cv.ShowImage("Fill", fill)
            # 
            #     cv.NamedWindow("H-thresh", 0)
            #     cv.ShowImage("H-thresh", h_plane)
            # 
            #     cv.Circle(sm, point, 10, (255,0,0),-1)
            #     cv.NamedWindow("Small", 0)
            #     cv.ShowImage("Small", sm)

def main(args):
      vn = hsv_node()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down hsv node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

