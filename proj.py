# Given camera params, create observation

import numpy as np
import cv2


# def proj_world_coord(fx, fy, cx, cy, T, alpha, point):
class camera:
    def __init__(self, ref_point_info):
        self.img_width = float(ref_point_info["EXIF"]["ExifImageWidth"])
        self.img_height = float(ref_point_info["EXIF"]["ExifImageHeight"])
        self.cx = self.img_width / 2
        self.cy = self.img_height / 2

        self.f = float(ref_point_info["EXIF"]["FocalLength"])
        # Calculate focal lengths in pixels
        self.sensor_width = 17.3  # in mm
        self.sensor_height = 13.0  # in mm
        self.f_x = (self.f * self.img_width) / self.sensor_width
        self.f_y = (self.f * self.img_height) / self.sensor_height
        self.alt_offset = float(ref_point_info["XMPInfo"]["RelativeAltitude"])

    def _get_image_corners(self):
        img_corners = np.array(
            [
                [0, 0],
                [self.img_width, 0],
                [0, self.img_height],
                [self.img_width, self.img_height],
            ]
        )
        return img_corners

    def _normalize_corners(self, img_corners=None):
        if img_corners == None:
            img_corners = self._get_image_corners()
        img_corners_normalized = np.array(
            [
                [(x - self.cx) / self.f_x, (y - self.cy) / self.f_y, 1.0]
                for x, y in img_corners
            ]
        )
        return img_corners_normalized

    def imgToWorldCoord(self, T, point_info):
        # Transform rays into world coordinates
        T[2] += self.alt_offset
        img_corners_normalized = self._normalize_corners()
        yaw, pitch, roll = (
            float(point_info["XMPInfo"]["FlightYawDegree"]),
            float(point_info["XMPInfo"]["FlightPitchDegree"]),
            float(point_info["XMPInfo"]["FlightRollDegree"]),
        )
        world_corners = []
        R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[0]
        ground_z = 0
        for corner in img_corners_normalized:
            ray_dir = np.dot(R, corner)  # Rotate into world coordinates
            scale = (ground_z - T[2]) / ray_dir[2]
            # Compute scale factor for intersection with ground
            world_point = T + scale * ray_dir  # Compute world coordinates
            world_corners.append(world_point)
        return world_corners
        # print(world_corners)
