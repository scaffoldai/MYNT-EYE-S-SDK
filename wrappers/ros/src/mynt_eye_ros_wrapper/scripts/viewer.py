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
    self.bridge = CvBridge()
    self.mynteye_head_subl = message_filters.Subscriber("mynteye_head/left_rect",Image)
    self.mynteye_head_subr = message_filters.Subscriber("mynteye_head/right_rect",Image)
    
    self.mynteye_cabin_subl = message_filters.Subscriber("mynteye_cabin/left_rect",Image)
    self.mynteye_cabin_subr = message_filters.Subscriber("mynteye_cabin/right_rect",Image)
    
    self.ts = message_filters.ApproximateTimeSynchronizer([
      self.mynteye_head_subl, self.mynteye_head_subr,
      self.mynteye_cabin_subl, self.mynteye_cabin_subr], 
      10, 0.1)
    self.ts.registerCallback(self.callback)

  def callback(self, headl, headr, cabinl, cabinr):
    try:
      cv_headl = self.bridge.imgmsg_to_cv2(headl, desired_encoding="passthrough")
      cv_headr = self.bridge.imgmsg_to_cv2(headr, desired_encoding="passthrough")
      cv_cabinl = self.bridge.imgmsg_to_cv2(cabinl, desired_encoding="passthrough")
      cv_cabinr = self.bridge.imgmsg_to_cv2(cabinr, desired_encoding="passthrough")
      
      head = np.hstack((cv_headl, cv_headr))
      cabin = np.hstack((cv_cabinl, cv_cabinr))
      combined = np.vstack((head, cabin))
    except CvBridgeError as e:
      print(e)
    cv2.imshow("Combined view", combined)
    cv2.waitKey(3)

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
