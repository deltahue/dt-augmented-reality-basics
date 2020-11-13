#!/usr/bin/env python3
import cv2
import numpy as np
import yaml
from sensor_msgs.msg import CameraInfo

class Augmenter:
    def __init__(self, veh_name, filename):
        
        self.intrinsics = self.load_camera_info(f'/data/config/calibrations/camera_intrinsic/{veh_name}.yaml')
        
        self.K = np.array(self.intrinsics.K)
        self.K = np.reshape(self.K, (3,-1))
        self.D = np.array(self.intrinsics.D)
        
        
        self.extrinsics = self.readYamlFile(f'/data/config/calibrations/camera_extrinsic/{veh_name}.yaml')
        self.homography = np.array(self.extrinsics['homography'])
        self.homography = np.reshape(self.homography, (3,-1))

        self.homography_inv = np.linalg.inv(self.homography)

        self.map = self.readYamlFile(f'/map/{filename}')

        self.pxl_points = self.ground2pixel(self.map['points'])

    def load_camera_info(self, filename):
        """Loads the camera calibration files.
        Loads the intrinsic camera calibration.
        Args:
            filename (:obj:`str`): filename of calibration files.
        Returns:
            :obj:`CameraInfo`: a CameraInfo message object
        """
        with open(filename, 'r') as stream:
            calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

    def readYamlFile(self,fname):
        """
        Reads the YAML file in the path specified by 'fname'.
        E.G. :
        the calibration file is located in : `/data/config/calibrations/camera_intrinsic/DUCKIEBOT_NAME.yaml`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return

    def process_image(self, img):

        img = cv2.undistort(img, self.K, self.D)
        return img

    def ground2pixel(self, points):
        # extract points in pixel coordinates
        pxl_points = {}
        for point in points:
            ground_point = np.array([points[point][1][0], points[point][1][1], 1.0])

            image_pt = np.dot(self.homography_inv, ground_point)

            pxl_points[point] = [int(image_pt[0]/image_pt[2]), int(image_pt[1]/image_pt[2])]

        return pxl_points

    
    def render_segments(self, img):
        
        segments = self.map['segments']
        #TODO: change to pxl points
        points = self.pxl_points

        for s in segments:
            pt_x = points[s['points'][0]]
            pt_y = points[s['points'][1]]
            color = s['color']
            img = self.draw_segment(img, pt_x, pt_y, color)

        return img

    def draw_segment(self, image, pt_x, pt_y, color):
        defined_colors = {
            'red': ['rgb', [1, 0, 0]],
            'green': ['rgb', [0, 1, 0]],
            'blue': ['rgb', [0, 0, 1]],
            'yellow': ['rgb', [1, 1, 0]],
            'magenta': ['rgb', [1, 0 , 1]],
            'cyan': ['rgb', [0, 1, 1]],
            'white': ['rgb', [1, 1, 1]],
            'black': ['rgb', [0, 0, 0]]}
        _color_type, [r, g, b] = defined_colors[color]
        cv2.line(image, tuple(pt_x), tuple(pt_y), (b * 255, g * 255, r * 255), 5)
        return image
