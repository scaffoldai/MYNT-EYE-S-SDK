#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('mynt_eye_ros_wrapper')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import numpy as np

class ImageViewer:

  def __init__(self):
    self.winname = 'Head view'
    cv2.namedWindow(self.winname)
    cv2.moveWindow(self.winname, 0, 0)

    self.bridge = CvBridge()
    self.mynteye_head_subr = rospy.Subscriber("mynteye_head/right_rect",Image, self.callback)

  def callback(self, headr):
    try:
      cv_headr = self.bridge.imgmsg_to_cv2(headr, desired_encoding="passthrough")
    except CvBridgeError as e:
      print(e)
    cv2.imshow(self.winname, cv_headr)
    cv2.waitKey(1)

def main(args):
  rospy.init_node('scaffold_mynteye_viewer', anonymous=True)
  ic = ImageViewer()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
