import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import numpy as np
import sys

class ImageConverter:
  def __init__(self, topic: str, depth: bool = False):
    self.depth = depth
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic, Image, self.callback)
    self.latest = None

  def callback(self, data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data)
      
      if self.depth:
        # Correct infinite and Nan values
        cv_image = np.nan_to_num(cv_image)

        # Normalize for depth  
        min = cv_image.min()
        cv_image = cv_image - min
        max = cv_image.max()
        cv_image = (cv_image / max)  * 255
        cv_image = cv_image.astype(np.uint8)
        
    except CvBridgeError as e:
      rospy.logerr(e)

    self.latest = cv_image
    #cv2.imshow("Image window", cv_image)
    #cv2.imwrite("ultima.png", cv_image)
    #cv2.waitKey(3)

    #try:
    #  self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    #except CvBridgeError as e:
    #  print(e)