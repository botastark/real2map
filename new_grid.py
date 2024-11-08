import os
import glob

# import cv2
import cv2
from matplotlib import pyplot as plt
import numpy as np

from proj import camera
from utilities import (
    extract_field_shape,
    extract_gps_data,
    get_image_properties,
    gps2ned,
    read_kmz_and_extract_coordinates,
)

from ploter import plot_3DGrid, plot_drone_fov_in_3d, plot_field_boundaries, plot_fov


DIR = "/media/bota/BOTA/wheat/APPEZZAMENTO PICCOLO/"

images, geodetic_data, geocentric_data, ned_data = [], [], [], []
images.extend(glob.glob(os.path.join(DIR, "*.JPG")))

# print(ref_point_info["EXIF"].keys())
# print(ref_point_info["EXIF"]["ExifImageHeight"])
# print(ref_point_info["EXIF"]["LensSpecification"])


for image_path in images:
    gps_info = get_image_properties(image_path)["GPSInfo"]
    xmp_info = get_image_properties(image_path)["XMPInfo"]
    geodetic_data.append(extract_gps_data(gps_info, xmp_info))


ned_data = gps2ned(geodetic_data, ref_id=0)
ref_geo = geodetic_data[0]
# plot_3DGrid(ned_data)
# ned1 = ned_data[0]
# ned2 = ned_data[1]
# distance_ned = np.linalg.norm(np.array(ned1) - np.array(ned2))


world_corners_all = []
ref_point_info = get_image_properties(images[0])

# ref_rel_alt = float(ref_point_info["XMPInfo"]["RelativeAltitude"])
L2 = camera(ref_point_info)
# point_ids = [11, 16, 215, 123, 47]
for point_id in range(len(images)):
    # for point_id in point_ids:
    img_info = get_image_properties(images[point_id])
    T = ned_data[point_id]
    world_corners = L2.imgToWorldCoord(T, img_info)
    world_corners_all.append(world_corners)
# plot_fov(world_corners_all[5])

# plot_drone_fov_in_3d(ned_data, world_corners_all)
field_corners = extract_field_shape(ned_data, world_corners_all)
# plot_field_boundaries(field_corners)
print(field_corners)

from scipy.spatial import ConvexHull


def minimum_bounding_rectangle(points):
    """
    Calculates the oriented minimum bounding rectangle of a set of 2D points.
    """
    # Get the convex hull
    hull_points = points[ConvexHull(points).vertices]

    # Use OpenCV to find the rotated rectangle that bounds the hull points
    rect = cv2.minAreaRect(hull_points.astype(np.float32))
    box = cv2.boxPoints(rect)  # Get the corner points of the box
    box = np.int32(box)

    return box


# Get the oriented bounding box
bounding_box = minimum_bounding_rectangle(field_corners)

# Plot original field and its bounding box
plt.figure(figsize=(8, 8))
plt.plot(field_corners[:, 0], field_corners[:, 1], "bo-", label="Field Corners")
plt.fill(field_corners[:, 0], field_corners[:, 1], color="lightblue", alpha=0.5)

# Draw the oriented bounding box
plt.plot(
    np.append(bounding_box[:, 0], bounding_box[0, 0]),
    np.append(bounding_box[:, 1], bounding_box[0, 1]),
    "r-",
    lw=2,
    label="Bounding Box",
)
plt.fill(bounding_box[:, 0], bounding_box[:, 1], color="yellow", alpha=0.3)

# Axis labels and title
plt.xlabel("X (North)")
plt.ylabel("Y (East)")
plt.title("Oriented Minimum Bounding Box for Field Corners")
plt.gca().set_aspect("equal", adjustable="box")
plt.legend()
plt.show()
"""
# exploring KMZ found
kmz_file_path = "/home/bota/Downloads/Adria_allettato.kmz"
gps_data = read_kmz_and_extract_coordinates(kmz_file_path)

ned_data_kmz = []
# Check the extracted data
geodetic_data_kmz = []
for i, placemark_coords in enumerate(gps_data):
    # print(f"Placemark {i+1} coordinates:")
    for coord in placemark_coords:
        # print(coord)  # Prints (latitude, longitude, altitude)
        geodetic_data_kmz.append([coord[0], coord[1], coord[1] + ref_rel_alt])
        # ecef_point = geodetic_to_ecef()
        # ned_data_kmz.append(ecef_to_ned(ecef_point, ref_geo[0], ref_geo[1], ref_geo[2]))
ned_data_kmz = gps2ned(geodetic_data_kmz, ref_geo=ref_geo)
ned_data.extend(ned_data_kmz)
plot_3DGrid(ned_data)
"""
