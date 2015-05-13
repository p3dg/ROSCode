#!/usr/bin/env python

import roslib
roslib.load_manifest('hsv_filter')
import sys
import rospy
import cv
import math 
import sys

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import LaserScan

#http://en.wikipedia.org/wiki/Bresenham's_line_algorithm
def bresenham_march(img, p1, p2):
 # function line(x0, x1, y0, y1)
 #     boolean steep = abs(y1 - y0) > abs(x1 - x0)
 #     if steep then
 #         swap(x0, y0)
 #         swap(x1, y1)
 #     if x0 > x1 then
 #         swap(x0, x1)
 #         swap(y0, y1)
 #     int deltax = x1 - x0
 #     int deltay = abs(y1 - y0)
 #     real error = 0
 #     real deltaerr = deltay / deltax
 #     int ystep
 #     int y = y0
 #     if y0 < y1 then ystep = 1 else ystep = -1
 #     for x from x0 to x1
 #         if steep then plot(y,x) else plot(x,y)
 #         error = error + deltaerr
 #         if error >= 0.5 then
 #             y = y + ystep
 #             error = error - 1.0
   x1 = p1[0]
   y1 = p1[1]
   x2 = p2[0]
   y2 = p2[1]
   steep = math.fabs(y2 - y1) > math.fabs(x2 - x1)
   if steep:
      t = x1
      x1 = y1
      y1 = t

      t = x2
      x2 = y2
      y2 = t
   also_steep = x1 > x2
   if also_steep:
      
      t = x1
      x1 = x2
      x2 = t

      t = y1
      y1 = y2
      y2 = t

   dx = x2 - x1
   dy = math.fabs(y2 - y1)
   error = 0.0
   delta_error = 0.0; # Default if dx is zero
   if dx != 0:
       delta_error = math.fabs(dy/dx)

   if y1 < y2:
      y_step = 1 
   else:
      y_step = -1

   y = y1
   ret = list([])
   for x in range(x1, x2):
      if steep:
         p = (y, x)
      else:
         p = (x, y)
      
      (b,r,g,a) = (-1,)*4
      if p[0] < img.width and p[1] < img.height:
         (b,r,g, a) = cv.Get2D(img, p[1], p[0])

      ret.append((p,(b,r,g)))

      error += delta_error
      if error >= 0.5:
         y += y_step
         error -= 1

   if also_steep:
       ret.reverse()

   return ret

class raycast_node:
    def __init__(self):
        rospy.init_node('raycast_node')

        """ Create the cv_bridge object """
        self.bridge = CvBridge()

        """ Subscribe to the raw camera image topic """
        self.image_sub = rospy.Subscriber("/static_image", Image, self.handle_image)
        self.image_pub = rospy.Publisher("/scan_image",Image)
        self.scan_pub = rospy.Publisher('/raycast', LaserScan)

        self.meters_per_pixel = .01
        self.radius = 300

        self.scan = LaserScan()
        self.scan.angle_min = math.pi /2
        self.scan.angle_max = - math.pi /2
        self.scan.angle_increment = math.pi / 180
        self.scan.range_min = 0
        self.scan.range_max = self.radius * self.meters_per_pixel

        # opencv has 0 at six oclock, 90 at three oclock, 180 at 12 oclock. Wierd.
        self.lookup = dict([(n, dict(sin = math.cos(math.radians(n)), cos=math.sin(math.radians(n)))) for n in range(90, 271)])

        for key, angle in  self.lookup.items():
           angle["x"] = self.radius * angle["cos"]
           angle["y"] = self.radius * angle["sin"]

    def handle_image(self, data):

        try:
            """ Convert the raw image to OpenCV format """
            src = self.bridge.imgmsg_to_cv(data, "bgr8")
            self.process_image(src)
        except CvBridgeError, e:
            print e
            return

    def process_image(self, img):
       (w,h) = cv.GetSize(img)
       self.camera_point = (w/2, h-1)
       ranges = []
       for key, angle in  self.lookup.items():
           p =  (int(angle["x"] + w/2), int(angle["y"] + h-1))
           for pixel in  bresenham_march(img, self.camera_point, p):
               p2 = pixel[0]
               (b,g,r) = pixel[1]
               if b == 0 and r == 0 and g == 0:
                  break           
           distance = math.hypot(self.camera_point[0]-p2[0], self.camera_point[1]-p2[1])
           if distance > self.radius:
              distance = self.radius -1
           ranges.append(distance * self.meters_per_pixel)
           cv.Line(img, self.camera_point, p2, (100,100,100))
       now = rospy.get_rostime()
       self.scan.header = Header()
       self.scan.header.frame_id = "camera"
       self.scan.header.stamp.secs = now.secs
       self.scan.header.stamp.nsecs = now.nsecs
       self.scan.ranges = ranges
       self.scan_pub.publish(self.scan)
       try:
          self.image_pub.publish(self.bridge.cv_to_imgmsg(img, "bgr8"))
       except CvBridgeError, e:
          print e

       cv.ShowImage("scan", img)

def main(args):
      vn = raycast_node()
      try:
        rospy.spin()
      except KeyboardInterrupt:
        print "Shutting down raycast node."
      cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

