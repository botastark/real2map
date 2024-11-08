from PIL.ExifTags import TAGS, GPSTAGS
from PIL import Image
from matplotlib import pyplot as plt
import pyproj
import navpy
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.cm as cm
from libxmp import *


wgs84 = pyproj.CRS("EPSG:4326")  # Geodetic (Lat, Lon, Alt) CRS
ecef = pyproj.CRS("EPSG:4978")  # ECEF CRS
# Define a transformer to convert geodetic to ECEF
geodetic_to_ecef_transformer = pyproj.Transformer.from_crs(wgs84, ecef, always_xy=True)


def _dms_to_dd(dms_tuple, ref):
    degrees = dms_tuple[0]
    minutes = float(dms_tuple[1] / 60)
    seconds = float(dms_tuple[2] / 3600)
    decimal_degrees = degrees + minutes + seconds
    if ref in ["S", "W"]:
        decimal_degrees = -decimal_degrees
    return decimal_degrees


def extract_gps_data(gps_info, xml_info):
    lat_dms = gps_info["GPSLatitude"]
    lat_ref = gps_info["GPSLatitudeRef"]
    lon_dms = gps_info["GPSLongitude"]
    lon_ref = gps_info["GPSLongitudeRef"]
    # altitude = gps_info["GPSAltitude"]
    altitude = float(xml_info["RelativeAltitude"])
    lat_dd = _dms_to_dd(lat_dms, lat_ref)  # Latitude in decimal degrees
    lon_dd = _dms_to_dd(lon_dms, lon_ref)  # Longitude in decimal degrees
    return [lat_dd, lon_dd, altitude]


# Define a function to convert geodetic to ECEF (Geocentric) using pyproj
def geodetic_to_ecef(lat, lon, alt):
    X, Y, Z = geodetic_to_ecef_transformer.transform(lon, lat, alt)
    return np.array([X, Y, Z])


# Function to convert from ECEF to NED using navpy
def ecef_to_ned(ecef_point, ref_lat, ref_lon, ref_alt):
    ned_point = navpy.ecef2ned(
        ecef_point,
        ref_lat,
        ref_lon,
        ref_alt,
        latlon_unit="deg",
    )
    return ned_point


def get_image_properties(image_path):
    img = Image.open(image_path)

    image_data = {}
    exif_data = img._getexif()

    xmp_data = img.getxmp()["xmpmeta"]["RDF"]["Description"]
    image_data["XMPInfo"] = xmp_data

    if exif_data:
        exif_info = {}
        for tag, value in exif_data.items():
            tag_name = TAGS.get(tag, tag)
            exif_info[tag_name] = value
        image_data["EXIF"] = exif_info

        # # Extract GPS info if available
        if "GPSInfo" in exif_info:
            gps_info = {}
            for key in exif_info["GPSInfo"].keys():
                gps_tag = GPSTAGS.get(key, key)
                gps_info[gps_tag] = exif_info["GPSInfo"][key]
            image_data["GPSInfo"] = gps_info
    return image_data


def gps2ned(geodetic_data, ref_id=0):
    geocentric_data, ned_data = [], []
    for lat1, lon1, alt1 in geodetic_data:
        geocentric_data.append(geodetic_to_ecef(lat1, lon1, alt1))

    ref_geo = geodetic_data[ref_id]
    for ecef1 in geocentric_data:
        ned_data.append(ecef_to_ned(ecef1, ref_geo[0], ref_geo[1], ref_geo[2]))

    # Coreection is necessary due to approx uncertainty
    correction_value = ned_data[ref_id]
    ned_data = [
        [
            data[0] - correction_value[0],
            data[1] - correction_value[1],
            data[2] - correction_value[2],
        ]
        for data in ned_data
    ]
    return ned_data


def plot_3DGrid(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ned = np.array(points)
    xs = ned[:, 0]
    ys = ned[:, 1]
    zs = ned[:, 2]

    num_points = xs.shape[0]

    scatter = ax.scatter(
        xs, ys, zs, c=np.arange(num_points), cmap="viridis", marker="o"
    )
    ax.scatter(xs[0], ys[0], zs[0], c="r", marker="x")
    ax.set_title("waypoints NED with reference (0,0) marked x(red)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label("Point Index")

    plt.show()


# https://stackoverflow.com/questions/67410270/how-to-draw-a-flat-3d-rectangle-in-matplotlib


def plot_fov(points):
    i = 0
    for point in points:
        # for each img there's 4 corners
        ned = np.array(point)
        xs = ned[:, 0]
        ys = ned[:, 1]
        zs = ned[:, 2]
        fig = plt.figure(figsize=(8, 8))
        ax = plt.axes(projection="3d")
        # ax.contourf(xs, ys, zs, cmap=cm.coolwarm)
        surf1 = ax.plot_trisurf(xs, ys, zs, antialiased=True)
        if i == 2:
            break
    plt.show()
    # break


from matplotlib.patches import Polygon


import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def order_fov_corners(corners):
    corners = np.array(corners)
    centroid = np.mean(corners[:, :2], axis=0)
    angles = np.arctan2(corners[:, 1] - centroid[1], corners[:, 0] - centroid[0])
    ordered_corners = corners[np.argsort(angles)]
    return ordered_corners


def plot_drone_fov_in_3d(ned_data, world_corners_all):
    """
    Plot drone positions with altitude and FoV projections on the ground plane in the NED frame (3D).

    Parameters:
    ned_data (numpy.ndarray): Array of drone positions (n, 3) in the NED frame.
    world_corners_all (list of lists): Each entry contains 4 (x, y, 0) tuples for the FoV corners on the ground.
    """
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    num_positions = len(ned_data)
    colors = cm.viridis(
        np.linspace(0, 1, num_positions)
    )  # You can choose other colormaps like 'plasma', 'jet', etc.

    # Ensure the lists are of equal length
    assert len(ned_data) == len(
        world_corners_all
    ), "Each camera position must have a corresponding FoV projection."

    # Iterate over each camera position and its corresponding FoV corners
    for idx, (loc, corners) in enumerate(zip(ned_data, world_corners_all)):
        # Plot the camera location in 3D
        corners_ = order_fov_corners(corners)
        color = colors[idx]
        ax.scatter(
            loc[0],
            loc[1],
            loc[2],
            color=color,
            s=50,
            label=f"Camera {idx + 1}" if idx == 0 else "",
        )

        # Create a 3D polygon for the FoV on the ground
        fov_polygon = Poly3DCollection(
            [corners_], color=color, alpha=0.3, edgecolor=color
        )
        ax.add_collection3d(fov_polygon)

        # Draw rays from the camera location to each corner of the FoV on the ground
        for corner in corners:
            ax.plot(
                [loc[0], corner[0]],
                [loc[1], corner[1]],
                [loc[2], corner[2]],
                color=color,
                linestyle="--",
                linewidth=1,
            )

    # Set axis labels and title
    ax.set_xlabel("X (North)")
    ax.set_ylabel("Y (East)")
    ax.set_zlabel("Z (Down)")
    ax.set_title("Drone Positions and Projected FoV on Ground with Ray Casting")
    # Create a ScalarMappable to map color values
    norm = plt.Normalize(vmin=0, vmax=num_positions - 1)
    sm = plt.cm.ScalarMappable(cmap="viridis", norm=norm)
    sm.set_array(
        []
    )  # Empty array as we're not displaying data but just the color scale

    # Add the colorbar to the plot
    cbar = plt.colorbar(sm, ax=ax, shrink=0.7, aspect=10)
    cbar.set_label("Camera Position Index", rotation=270, labelpad=15)

    # Adjust the view to show the 3D structure
    ax.view_init(elev=45, azim=135)
    ax.grid(True)
    ax.legend()
    plt.show()


import numpy as np
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt


def extract_field_shape(ned_data, world_corners_all):
    """
    Extracts the field boundary based on the drone camera positions and their projected FoV corners.
    For rectangular fields, the corners are calculated directly. For non-rectangular fields, we calculate the convex hull.

    Parameters:
    ned_data (numpy.ndarray): Array of drone positions (n, 3) in the NED frame.
    world_corners_all (list of lists): List of FoV corners for each position, where each entry has 4 (x, y, 0) tuples.

    Returns:
    field_corners (np.ndarray): Corners of the field (approximated or exact).
    """
    all_corners = []

    # Iterate over each camera position and its corresponding FoV corners
    for corners in world_corners_all:
        # Flatten corners into a 2D list of (x, y) coordinates (ignoring z=0)
        for corner in corners:
            all_corners.append(corner[:2])  # Only take x, y

    # Convert all corners into a numpy array
    all_corners = np.array(all_corners)

    # If the field is rectangular, we could use an approach that simply finds the min/max x and y values:
    x_min, y_min = np.min(all_corners, axis=0)
    x_max, y_max = np.max(all_corners, axis=0)

    # Assuming the field is rectangular, the corners would be:
    rect_corners = np.array(
        [[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min]]
    )

    # If the field is non-rectangular, compute the convex hull of the corners
    try:
        hull = ConvexHull(all_corners)
        hull_corners = all_corners[hull.vertices]  # Get the corners of the convex hull
        return hull_corners
    except:
        # If convex hull cannot be computed (e.g., due to degenerate case), return the rectangular corners
        print("Could not compute convex hull, returning rectangular field corners.")
        return rect_corners


# plt.title('Extracted Field Boundary')
# plt.show()
import zipfile
import xml.etree.ElementTree as ET


def read_kmz_and_extract_coordinates(kmz_file_path):
    """
    Reads a KMZ file, extracts GPS coordinates from each Placemark, and stores them in a list.

    Parameters:
    kmz_file_path (str): Path to the KMZ file.

    Returns:
    list: A list of GPS coordinates (latitude, longitude, altitude) for each Placemark.
    """
    gps_coordinates = []  # List to store GPS coordinates

    # Open the KMZ file as a zip archive
    with zipfile.ZipFile(kmz_file_path, "r") as kmz:
        # Find the KML file within the KMZ
        kml_filename = [name for name in kmz.namelist() if name.endswith(".kml")][0]

        # Extract and parse the KML file
        with kmz.open(kml_filename) as kml_file:
            tree = ET.parse(kml_file)
            root = tree.getroot()

            # Parse the KML data and extract coordinates from each Placemark
            for placemark in root.iter("{http://www.opengis.net/kml/2.2}Placemark"):
                coordinates_text = placemark.find(
                    ".//{http://www.opengis.net/kml/2.2}coordinates"
                ).text.strip()

                # Split the coordinates and store them as (latitude, longitude, altitude)
                placemark_coords = []
                for coord in coordinates_text.split():
                    lon, lat, alt = map(float, coord.split(","))
                    placemark_coords.append(
                        (lat, lon, alt)
                    )  # Store as (latitude, longitude, altitude)

                gps_coordinates.append(
                    placemark_coords
                )  # Store all coords for this Placemark as a sublist

    return gps_coordinates
