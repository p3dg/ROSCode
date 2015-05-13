#!/usr/bin/env python

import roslib
roslib.load_manifest('line_filter')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class LineFilterNode:

    def __init__(self):
        rospy.init_node('line_filter')

        """ Give the OpenCV display window a name. """
        self.input_window = "Input Image"
        self.output_window = "Output Image"

        self.calibrated = False

        """ Create the window and make it re-sizeable (second parameter = 0) """
        cv.NamedWindow(self.input_window, 0)
        cv.NamedWindow(self.output_window, 0)

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("/input_image", Image, self.handle_image)
        self.image_pub = rospy.Pubscriber(Image)

        self.outImg = cv.CreateImage((160, 120), cv.IPL_DEPTH_8U, 3);
        self.bwImg  = cv.CreateImage(cv.GetSize(self.), cv.IPL_DEPTH_8U, 1)

    def handle_image(self, data):

        try:
            """ Convert the raw image to OpenCV format """
            inImg = self.bridge.imgmsg_to_cv(data, "bgr8")
            (width, height) = cv.GetSize(inImg)

            cv.SetZero(self.outImg)
            cv.Resize(inImg, self.outImg)

            cv.PyrMeanShiftFiltering(self.outImg, self.outImg, 10, 20)

            cv.CvtColor(self.outImg,self.bwImg,cv.CV_BGR2GRAY)
            cv.Threshold(self.bwImg, self.bwimg, 150, 255, cv.CV_THRESH_BINARY_INV);

            cv.ShowImage(self.output_window, self.bwImg)

        except CvBridgeError, e:
            print e
            return
  
        cv.ShowImage(self.input_window, inImg)
        cv.WaitKey(3)

def main(args):
      lf = LineFilterNode()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down vison node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

