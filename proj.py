# Given camera params, create observation

import numpy as np
import cv2


class camera:
    def __init__(self, ref_point_info):
        self.img_width = float(ref_point_info["EXIF"]["ExifImageWidth"])
        self.img_height = float(ref_point_info["EXIF"]["ExifImageHeight"])
        self.cx = self.img_width / 2
        self.cy = self.img_height / 2

        self.f = float(ref_point_info["EXIF"]["FocalLength"])
        self.pixel_size = 3.3e-3  # e-6, to have width in mm
        self.sensor_width, self.sensor_height = (
            self.img_width * self.pixel_size,
            self.img_height * self.pixel_size,
        )

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
            np.deg2rad(float(point_info["XMPInfo"]["GimbalYawDegree"])),
            np.deg2rad(float(point_info["XMPInfo"]["GimbalPitchDegree"])),
            np.deg2rad(float(point_info["XMPInfo"]["GimbalRollDegree"])),
        )
        world_corners = []
        # R_z = cv2.Rodrigues(np.array([0, 0, yaw]))[0]
        # R_y = cv2.Rodrigues(np.array([0, pitch, 0]))[0]
        # R_x = cv2.Rodrigues(np.array([roll, 0, 0]))[0]
        # R = R_z @ R_y @ R_x  # Combine rotations in ZYX order
        R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[0]
        ground_z = 0
        for corner in img_corners_normalized:
            ray_dir = R @ corner  # Rotate corner ray into world coordinates
            scale = (ground_z - T[2]) / ray_dir[2]
            # Compute scale factor for intersection with ground
            world_point = T + scale * ray_dir  # Compute world coordinates
            world_corners.append(world_point)

        return world_corners
