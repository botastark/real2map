import os
import glob

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
    read_kmz_and_extract_coordinates,
)

from ploter import (
    plot_3DGrid,
    plot_drone_fov_in_3d,
    plot_fov_2d,
    plot_tiles,
)


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


world_corners_all = []
ref_point_info = get_image_properties(images[0])

# ref_rel_alt = float(ref_point_info["XMPInfo"]["RelativeAltitude"])
L2 = camera(ref_point_info)
for point_id in range(len(images)):
    img_info = get_image_properties(images[point_id])
    T = ned_data[point_id]
    world_corners = np.array(L2.imgToWorldCoord(T, img_info))
    world_corners_all.append(world_corners)
corners = four_corners(world_corners_all, 30)
print(corners)
field_corners = extract_field_shape(ned_data, world_corners_all)
bounding_box = minimum_bounding_rectangle(field_corners)
# plot_fov_2d(world_corners_all, corners, field_corners, bounding_box)
# plot_drone_fov_in_3d(ned_data, world_corners_all)


rows, cols = 10, 10
tiles = divide_field_into_tiles(corners, (rows, cols))

plot_tiles(tiles, corners)


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
