import math
from matplotlib import pyplot as plt
import numpy as np
from conversion import Converter
import os
import glob
from PIL import Image

# import piexif
from PIL.ExifTags import TAGS, GPSTAGS
from mpl_toolkits.mplot3d import Axes3D


def get_image_properties(image_path):
    img = Image.open(image_path)

    image_data = {}

    # image_data["Format"] = img.format
    # image_data["Mode"] = img.mode
    # image_data["Size"] = img.size  # (width, height)
    # image_data["Info"] = img.info

    exif_data = img._getexif()

    if exif_data:
        # Initialize a dictionary for EXIF data
        exif_info = {}
        for tag, value in exif_data.items():
            tag_name = TAGS.get(tag, tag)
            exif_info[tag_name] = value
        # image_data["EXIF"] = exif_info

        # # Extract GPS info if available
        if "GPSInfo" in exif_info:
            gps_info = {}
            for key in exif_info["GPSInfo"].keys():
                gps_tag = GPSTAGS.get(key, key)
                gps_info[gps_tag] = exif_info["GPSInfo"][key]
            image_data["GPSInfo"] = gps_info

    return image_data


DIR = "/home/bota/Desktop/wheat/APPEZZAMENTO PICCOLO/"

images = []
images.extend(glob.glob(os.path.join(DIR, "*.JPG")))

gps_info_ref = get_image_properties(images[0])["GPSInfo"]
conv = Converter(gps_info_ref)
points = []
raw_points = []
for image_path in images:
    gps_info = get_image_properties(image_path)["GPSInfo"]
    print(conv.extract_gps_data(gps_info))
    point = conv.ecef2ned(
        conv.geodetic2ecef(conv.extract_gps_data(gps_info).inrad())
    ).to_ENU()
    points.append(point)
    raw_points.append(conv.extract_gps_data(gps_info))
    # print(point)

dist = math.sqrt((points[0].x - points[-1].x) ** 2 + (points[0].y - points[-1].y) ** 2)
print(points[0])
print(points[-1])
print(dist)


# Function to plot WRF_point objects in 3D
def plot_WRF_points_3D(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")

    xs = [point.x for point in points]
    ys = [point.y for point in points]
    zs = [point.z for point in points]
    num_points = len(points)
    print(num_points)
    print(range(num_points))
    # Create a gradient color based on the number of points using a colormap
    num_points = len(points)
    colors = plt.cm.viridis(
        np.linspace(0, 1, num_points)
    )  # Use 'viridis' colormap for gradient

    # Plot the points as a 3D scatter plot with gradient colors
    scatter = ax.scatter(
        xs, ys, zs, c=np.arange(num_points), cmap="viridis", marker="o"
    )

    # ax.scatter(xs, ys, zs, c="r", marker="o")
    ax.scatter(xs[0], ys[0], zs[0], c="r", marker="x")
    ax.scatter(xs[-1], ys[-1], zs[-10], c="r", marker="x")

    # Set labels
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    # ax.set_zlim([0, 35])
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Point Index")
    # cbar.set_ticks(np.arange(num_points))  # Set ticks from 0 to num_points-1
    # cbar.set_ticklabels(np.arange(num_points))  # Set labels to match the indices

    plt.show()


# Plot the points in 3D
plot_WRF_points_3D(points)


# Extract latitudes, longitudes, and altitudes
lats = [point.lat for point in raw_points]
lons = [point.lon for point in raw_points]
alts = [point.alt for point in raw_points]

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Plot the GPS points
ax.scatter(lons, lats, alts, c="r", marker="o")

# Labels and plot details
ax.set_xlabel("Longitude")
ax.set_ylabel("Latitude")
ax.set_zlabel("Altitude (m)")
ax.set_title("3D Plot of GPS Data Points")

# Display the plot
plt.show()
