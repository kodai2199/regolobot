import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
import cv2
import numpy as np
from pathlib import Path


class ImageConverter:
    def __init__(
        self, topic: str, output_dir: Path, depth: bool = False, counter: int = 0
    ):
        self.output_dir = output_dir
        self.depth = depth
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(topic, Image, self.callback)
        self.latest = None
        self.counter = counter

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            rospy.logerr(e)
        if self.depth:
            # Correct infinite and Nan values
            cv_image = np.nan_to_num(cv_image, posinf=0, neginf=0)
            # Normalize for depth
            pixel_min = cv_image.min()
            cv_image = cv_image - pixel_min
            pixel_max = cv_image.max()
            cv_image = (cv_image / pixel_max) * 255
            cv_image = cv_image.astype(np.uint8)
        else:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.latest = cv_image

    def save(self):
        output_path = str(self.output_dir / f"{self.counter}.png")
        cv2.imwrite(output_path, self.latest)
        self.counter += 1
        return self.counter
