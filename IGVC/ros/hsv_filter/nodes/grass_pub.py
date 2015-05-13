#!/usr/bin/env python
import roslib
roslib.load_manifest('hsv_filter')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class grass_pub:

  def __init__(self):
    rospy.init_node('grass_pub', anonymous=True)
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher("/static_image",Image)
    self.static_image = cv.LoadImage("./1_distance_test.png")
    r = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        try:
          self.image_pub.publish(self.bridge.cv_to_imgmsg(self.static_image, "bgr8"))
        except CvBridgeError, e:
          print e
        r.sleep()

def main(args):
  ic = grass_pub()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
