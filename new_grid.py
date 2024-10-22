import os
import glob

# import cv2
import numpy as np

from proj import proj_world_coord
from utilities import (
    ecef_to_ned,
    extract_gps_data,
    geodetic_to_ecef,
    get_image_properties,
    plot_3DGrid,
)


DIR = "/home/bota/Desktop/wheat/APPEZZAMENTO PICCOLO/"

images, geodetic_data, geocentric_data, ned_data = [], [], [], []
images.extend(glob.glob(os.path.join(DIR, "*.JPG")))
ref_point_info = get_image_properties(images[0])
# print(ref_point_info["EXIF"].keys())

# print(ref_point_info["XMPInfo"])
# print(ref_point_info["EXIF"]["ExifImageHeight"])
# print(ref_point_info["EXIF"]["LensSpecification"])

ref_rel_alt = float(ref_point_info["XMPInfo"]["RelativeAltitude"])

for image_path in images:
    gps_info = get_image_properties(image_path)["GPSInfo"]
    xmp_info = get_image_properties(image_path)["XMPInfo"]
    geodetic_data.append(extract_gps_data(gps_info, xmp_info))

for lat1, lon1, alt1 in geodetic_data:
    geocentric_data.append(geodetic_to_ecef(lat1, lon1, alt1))

ref_geo = geodetic_data[0]
# print(ref_geo)
for ecef1 in geocentric_data:
    ned_data.append(ecef_to_ned(ecef1, ref_geo[0], ref_geo[1], ref_geo[2]))

# Coreection is necessary due to approx uncertainty
correction_value = ned_data[0]
ned_data = [
    [
        data[0] - correction_value[0],
        data[1] - correction_value[1],
        data[2] - correction_value[2],
    ]
    for data in ned_data
]


ned1 = ned_data[0]
ned2 = ned_data[1]
distance_ned = np.linalg.norm(np.array(ned1) - np.array(ned2))
# Output the results
# print(f"First point NED: {ned1}")
# print(f"Second point NED: {ned2}")
# print(f"Distance between points in NED frame: {distance_ned} meters")
# print(f"X Distance between points in NED frame: {ned1[0] - ned2[0]} meters")
# print(f"Y Distance between points in NED frame: {ned1[1] - ned2[1]} meters")
# print(f"Z Distance between points in NED frame: {ned1[2] - ned2[2]} meters")

plot_3DGrid(ned_data)
# print("ned data shaoe ", ned_data.shape)

img_width = float(ref_point_info["EXIF"]["ExifImageWidth"])
img_height = float(ref_point_info["EXIF"]["ExifImageHeight"])
cx = img_width / 2
cy = img_height / 2
yaw, pitch, roll = (
    float(ref_point_info["XMPInfo"]["FlightYawDegree"]),
    float(ref_point_info["XMPInfo"]["FlightPitchDegree"]),
    float(ref_point_info["XMPInfo"]["FlightRollDegree"]),
)
f = float(ref_point_info["EXIF"]["FocalLength"])
# Calculate focal lengths in pixels
sensor_width = 17.3  # in mm
sensor_height = 13.0  # in mm
f_x = (f * img_width) / sensor_width
f_y = (f * img_height) / sensor_height

ned2[2] += ref_rel_alt
print(ned2)
T = ned2
alpha = roll, pitch, yaw
img_corners = np.array(
    [[0, 0], [img_width, 0], [0, img_height], [img_width, img_height]]
)
# world_corner = []
# for corner in img_corners:
#     world_corner.append(proj_world_coord(f_x, f_y, cx, cy, T, alpha, corner))

# Convert image corners to normalized image coordinates
img_corners_normalized = np.array(
    [[(x - cx) / f_x, (y - cy) / f_y, 1.0] for x, y in img_corners]
)
# Transform rays into world coordinates
world_corners = []
# R = cv2.Rodrigues(np.array([roll, pitch, yaw]))[0]
# ground_z = 0
# for corner in img_corners_normalized:
#     ray_dir = np.dot(R, corner)  # Rotate into world coordinates
#     scale = (ground_z - T[2]) / ray_dir[
#         2
#     ]  # Compute scale factor for intersection with ground
#     world_point = T + scale * ray_dir  # Compute world coordinates
#     world_corners.append(world_point)

# # world_corners = np.array(world_corners)
# print(world_corners)
# print("world_corners data shaoe ", world_corners.shape)

# plot_3DGrid(world_corners)
