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


fov_corners_all = []
ref_point_info = get_image_properties(images[0])

# ref_rel_alt = float(ref_point_info["XMPInfo"]["RelativeAltitude"])
L2 = camera(ref_point_info)
for img_id in range(len(images)):
    img_info = get_image_properties(images[img_id])
    T = ned_data[img_id]
    fov_corners = np.array(L2.imgToWorldCoord(T, img_info))
    fov_corners_all.append(fov_corners)
corners = four_corners(fov_corners_all, 30)
# print(corners)
field_corners = extract_field_shape(ned_data, fov_corners_all)
bounding_box = minimum_bounding_rectangle(field_corners)
# plot_fov_2d(fov_corners_all, corners, field_corners, bounding_box)
# plot_drone_fov_in_3d(ned_data, fov_corners_all)


tile_size = (10, 10)
tiles = divide_field_into_tiles(corners, tile_size)
# print(fov_corners_all[0])
# print(tiles[0])
# plot_tiles(tiles, corners)


import matplotlib.pyplot as plt

tile_corners_ned = tiles

# Image dimensions (for example)
image_width, image_height = 5280, 3956  # pixels

# Example FOV NED coordinates (for mapping)
ned_corners = np.array(fov_corners_all[2][:, 0:2])
# print(ned_corners.shape)


def ned_to_image_mapping(ned_corners, image_width, image_height):
    """
    Maps the NED coordinates to image coordinates (pixels).
    """
    north_min, east_min = np.min(ned_corners, axis=0)
    north_max, east_max = np.max(ned_corners, axis=0)

    # Calculate pixel per NED unit in both directions
    pixel_per_north = image_height / (north_max - north_min)
    pixel_per_east = image_width / (east_max - east_min)

    return north_min, east_min, pixel_per_north, pixel_per_east


def map_tile_to_image(
    tile_corners, north_min, east_min, pixel_per_north, pixel_per_east
):
    """
    Maps a given tile's NED corners to image pixel coordinates.
    """
    # Convert NED coordinates to image coordinates (pixels)
    image_tile = []
    for corner in tile_corners:
        north, east = corner
        x = (east - east_min) * pixel_per_east
        y = (north_min - north) * pixel_per_north  # invert y-axis (top-left origin)
        image_tile.append([x, y])
    return np.array(image_tile)


def plot_image_with_tiles(
    image, tile_corners, north_min, east_min, pixel_per_north, pixel_per_east
):
    """
    Plots the image and overlays the tiles based on the given NED corners.
    """
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.imshow(image, cmap="gray")  # Assuming a grayscale image

    for corners in tile_corners:
        # Map the NED corners of the tile to image pixel coordinates
        image_tile = map_tile_to_image(
            corners, north_min, east_min, pixel_per_north, pixel_per_east
        )
        # print(image_tile)

        # Plot the tile boundary (connect the corners)
        image_tile = np.vstack(
            [image_tile, image_tile[0]]
        )  # Close the tile by adding the first corner
        ax.plot(image_tile[:, 0], image_tile[:, 1], color="r", alpha=0.7)
        ax.fill(image_tile[:, 0], image_tile[:, 1], alpha=0.2, color="b")
    ax.set_xlim((-2000, 6000))
    ax.set_ylim((-2000, 6000))
    ax.set_xlabel("East (pixels)")
    ax.set_ylabel("North (pixels)")
    ax.set_aspect("equal", "box")
    plt.show()


# Example image (randomly generated for illustration purposes)
image = np.random.rand(image_height, image_width)

# Map the NED FOV coordinates to image coordinates
north_min, east_min, pixel_per_north, pixel_per_east = ned_to_image_mapping(
    ned_corners, image_width, image_height
)

print(f"pixel per N E {pixel_per_north} - {pixel_per_east}")
# Plot the image with the tiles
plot_image_with_tiles(
    image, tile_corners_ned, north_min, east_min, pixel_per_north, pixel_per_east
)


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
