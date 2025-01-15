import os
import glob

import cv2

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH")
import numpy as np

from proj import camera
from utilities import (
    divide_field_into_tiles,
    extract_field_shape,
    extract_gps_data,
    four_corners,
    get_image_properties,
    gps2ned,
    minimum_bounding_rectangle,
    ned_to_image_mapping,
    read_kmz_and_extract_coordinates,
)

from ploter import (
    plot_3DGrid,
    plot_drone_fov_in_3d,
    plot_fov_2d,
    plot_image_with_tiles,
    plot_tiles,
)


DIR = "/media/bota/BOTA/wheat/APPEZZAMENTO_PICCOLO/"

images, geodetic_data, geocentric_data, ned_data = [], [], [], []
images.extend(glob.glob(os.path.join(DIR, "*.JPG")))

# print(ref_point_info["EXIF"].keys())
# print(ref_point_info["EXIF"]["ExifImageHeight"])
# print(ref_point_info["EXIF"]["LensSpecification"])


for image_path in images:
    gps_info = get_image_properties(image_path)["GPSInfo"]
    xmp_info = get_image_properties(image_path)["XMPInfo"]
    geodetic_data.append(extract_gps_data(gps_info, xmp_info))

ned_data = gps2ned(geodetic_data, ref_id=1)
ref_geo = geodetic_data[0]
# plot_3DGrid(ned_data)


fov_corners_all = []
ref_point_info = get_image_properties(images[0])
print(f"ref imag path {images[0]}")

# ref_rel_alt = float(ref_point_info["XMPInfo"]["RelativeAltitude"])
L2 = camera(ref_point_info)
for img_id in range(len(images)):
    img_info = get_image_properties(images[img_id])
    T = ned_data[img_id]
    fov_corners = np.array(L2.imgToWorldCoord(T, img_info))
    # fov_corners = np.array(L2.get_fov_corners_in_ned(T, img_info))
    fov_corners_all.append(fov_corners)
corners = four_corners(fov_corners_all, 30)
# field_corners = extract_field_shape(ned_data, fov_corners_all)
# bounding_box = minimum_bounding_rectangle(field_corners)
# plot_fov_2d(fov_corners_all, corners, field_corners, bounding_box)
plot_drone_fov_in_3d(ned_data[0:10], fov_corners_all[0:10])


# tile_size = (10, 10)
# tiles = divide_field_into_tiles(corners, tile_size)
# plot_tiles(tiles, corners)


# tile_corners_ned = tiles
test_id = 0
image_width, image_height = 5280, 3956  # pixels

ned_corners = np.array(fov_corners_all[test_id][:, 0:2])
print(ned_corners)
image = np.random.rand(image_height, image_width)
img_corners = np.array(
    [
        [0, 0],
        [0, image_height],
        [image_width, 0],
        [image_width, image_height],
    ]
)


# img_src = cv2.imread(images[test_id])

# matrix, _ = cv2.findHomography(img_corners, ned_corners, 0)
# print(matrix)
# perspective_img = cv2.warpPerspective(img_src, matrix, (image_height, image_width))


# import matplotlib.pyplot as plt

# image_rgb = cv2.cvtColor(
#     img_src, cv2.COLOR_BGR2RGB
# )  # Convert BGR to RGB for matplotlib
# plt.imshow(image_rgb)
# plt.axis("off")  # Hide axis
# plt.show()

# image_rgb = cv2.cvtColor(
#     perspective_img, cv2.COLOR_BGR2RGB
# )  # Convert BGR to RGB for matplotlib
# plt.imshow(image_rgb)
# plt.axis("off")  # Hide axis
# plt.show()


# # Map the NED FOV coordinates to image coordinates
# north_min, east_min, pixel_per_north, pixel_per_east = ned_to_image_mapping(
#     ned_corners, image_width, image_height
# )

# plot_image_with_tiles(
#     image, tile_corners_ned, north_min, east_min, pixel_per_north, pixel_per_east
# )
