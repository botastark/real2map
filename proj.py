# Given camera params, create observation

import math
import numpy as np
import cv2

# # Field of View in degrees for both modes
fov_horizontal = 70  # same for both modes
fov_vertical_non_repetitive = 75
import math

# Given parameters
focal_length = 12.29  # in mm
img_width_pixels = 5280
img_height_pixels = 3956


# Conversion function for sensor dimensions using FOV and focal length
def calculate_sensor_dim(focal_length, fov_horizontal, fov_vertical):
    sensor_width = 2 * focal_length * math.tan(math.radians(fov_horizontal / 2))
    sensor_height_repetitive = (
        2 * focal_length * math.tan(math.radians(fov_vertical / 2))
    )
    return sensor_width, sensor_height_repetitive


sensor_width, sensor_height_non_repetitive = calculate_sensor_dim(
    focal_length, fov_horizontal, fov_vertical_non_repetitive
)

print(
    f"sensor_width {sensor_width}h, sensor_height_repetitive {sensor_height_non_repetitive}"
)


class camera:
    def __init__(self, ref_point_info):
        self.img_width = float(ref_point_info["EXIF"]["ExifImageWidth"])
        self.img_height = float(ref_point_info["EXIF"]["ExifImageHeight"])
        self.cx = self.img_width / 2
        self.cy = self.img_height / 2

        self.f = float(ref_point_info["EXIF"]["FocalLength"])
        print(self.f)
        # self.f = 12.3
        # Calculate focal lengths in pixels
        self.sensor_width, self.sensor_height = calculate_sensor_dim(
            self.f, fov_horizontal, fov_vertical_non_repetitive
        )
        print(
            f"sensor_widt {self.sensor_width}h, sensor_height_repetitive {self.sensor_height}"
        )
        # self.sensor_width = 17.46  # in mm
        # self.sensor_height = 35.46  # in mm

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
        R_z = cv2.Rodrigues(np.array([0, 0, yaw]))[0]
        R_y = cv2.Rodrigues(np.array([0, pitch, 0]))[0]
        R_x = cv2.Rodrigues(np.array([roll, 0, 0]))[0]
        R = R_z @ R_y @ R_x  # Combine rotations in ZYX order
        # R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[0]
        ground_z = 0
        for corner in img_corners_normalized:
            ray_dir = R @ corner  # Rotate corner ray into world coordinates
            # ray_dir = np.dot(R, corner)  # Rotate into world coordinates
            scale = (ground_z - T[2]) / ray_dir[2]
            # Compute scale factor for intersection with ground
            world_point = T + scale * ray_dir  # Compute world coordinates
            world_corners.append(world_point)

        return world_corners
        # print(world_corners)
