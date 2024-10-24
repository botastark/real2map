import os
import glob

# import cv2
import cv2
import numpy as np

from proj import camera
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

# plot_3DGrid(ned_data)

world_corners_all = []
L2 = camera(ref_point_info)
point_id = 2
for point_id in range(len(images)):
    img_info = get_image_properties(images[point_id])
    T = ned_data[point_id]

    world_corners = L2.imgToWorldCoord(T, img_info)

    world_corners_all.append(world_corners)
# plot_3DGrid(world_corners_all)
print(np.array(world_corners).shape)
