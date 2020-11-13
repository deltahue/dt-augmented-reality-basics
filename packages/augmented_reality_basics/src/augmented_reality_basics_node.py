#!/usr/bin/env python3
import os
import rospy
import sys

from duckietown.dtros import DTROS, NodeType
from sensor_msgs.msg import CompressedImage, CameraInfo


import cv2
from cv_bridge import CvBridge
import numpy as np

# local import
import augmenter

class AugmentedRealityNode(DTROS):
    def __init__(self, node_name, map_file):
        # initialize the DTROS parent class
        super(AugmentedRealityNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh_name = rospy.get_namespace().strip("/")
        self.log("Say hello from AR-Node to " + self.veh_name)
        self.log(f"Using map file: {map_file}")

        self.bridge = CvBridge()


        # get yaml
        base_file_name = map_file[:-5]

        self._augmenter = augmenter.Augmenter(self.veh_name, map_file)
    

        # image subscriber
        self.sub_camera_image = rospy.Subscriber(f'/{self.veh_name}/camera_node/image/compressed', CompressedImage, self.callback)

        # image publisher
        self.pub_map_image = rospy.Publisher(f'/{self.veh_name}/{node_name}/{base_file_name}/image/compressed', CompressedImage, queue_size=10)


        self.log("Initialized")

    def callback(self, msg):
        """
        Get image
        """

        img= self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # perform undistort
        img = self._augmenter.process_image(img)

        # render segments
        img = self._augmenter.render_segments(img)

        # make compressed image and stamp
        img_msg = self.bridge.cv2_to_compressed_imgmsg(img)
        self.pub_map_image.publish(img_msg)

    
            
if __name__ == '__main__':
    map_file = sys.argv[1]
    node = AugmentedRealityNode(node_name='augmented_reality_basics_node', map_file=map_file)
    # Keep it spinning to keep the node alive
    rospy.spin()
    rospy.loginfo("augmented_reality_basics_node is up and running...")