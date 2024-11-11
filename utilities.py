from PIL.ExifTags import TAGS, GPSTAGS
from PIL import Image
import cv2
import pyproj
import navpy
import numpy as np
import xml.etree.ElementTree as ET

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


def gps2ned(geodetic_data, ref_id=0, ref_geo=None):
    geocentric_data, ned_data = [], []
    for lat1, lon1, alt1 in geodetic_data:
        geocentric_data.append(geodetic_to_ecef(lat1, lon1, alt1))
    if ref_geo is None:
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


import zipfile
import xml.etree.ElementTree as ET


def read_kmz_and_extract_coordinates(kmz_file_path):

    gps_coordinates = []  # List to store GPS coordinates

    with zipfile.ZipFile(kmz_file_path, "r") as kmz:
        kml_filename = [name for name in kmz.namelist() if name.endswith(".kml")][0]

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


def order_fov_corners(corners):
    corners = np.array(corners)
    centroid = np.mean(corners[:, :2], axis=0)
    angles = np.arctan2(corners[:, 1] - centroid[1], corners[:, 0] - centroid[0])
    ordered_corners = corners[np.argsort(angles)]
    return ordered_corners


def average_of_top_n(values, n, ascending=True):
    sorted_indices = np.argsort(values)
    if ascending:
        return np.median(values[sorted_indices[:n]])
    else:
        return np.median(values[sorted_indices[-n:]])


def four_corners(fovs, n_avg):
    fovs_ = np.array(fovs)
    avg_N = fovs_[:, :, 0].mean()
    avg_E = fovs_[:, :, 1].mean()
    quarter1, quarter2, quarter3, quarter4 = [], [], [], []
    fovs_ = fovs_.reshape(-1, 3)
    for fov in fovs_:
        if fov[0] < avg_N and fov[1] < avg_E:
            quarter3.append(fov)
        elif fov[0] < avg_N and fov[1] > avg_E:
            quarter4.append(fov)
        elif fov[0] > avg_N and fov[1] > avg_E:
            quarter1.append(fov)
        elif fov[0] > avg_N and fov[1] < avg_E:
            quarter2.append(fov)
        else:
            continue
    # max -max UR
    quarter1 = np.array(quarter1)

    upright = [
        average_of_top_n(quarter1[:, 0], n_avg, ascending=False),
        average_of_top_n(quarter1[:, 1], n_avg, ascending=False),
    ]

    #  max min UL
    quarter2 = np.array(quarter2)
    upleft = [
        average_of_top_n(quarter2[:, 0], n_avg, ascending=False),
        average_of_top_n(quarter2[:, 1], n_avg),
    ]
    # min-min DL
    quarter3 = np.array(quarter3)
    downleft = [
        average_of_top_n(quarter3[:, 0], n_avg),
        average_of_top_n(quarter3[:, 1], n_avg),
    ]
    #  min max DR
    quarter4 = np.array(quarter4)
    downright = [
        average_of_top_n(quarter4[:, 0], n_avg),
        average_of_top_n(quarter4[:, 1], n_avg, ascending=False),
    ]

    return np.array([upright, upleft, downleft, downright])


from scipy.spatial import ConvexHull


def minimum_bounding_rectangle(points):
    hull_points = points[ConvexHull(points).vertices]
    rect = cv2.minAreaRect(hull_points.astype(np.float32))
    box = cv2.boxPoints(rect)  # Get the corner points of the box
    box = np.int32(box)
    return box


def divide_field_into_tiles(corners, tile_size):
    x = corners[:, 0]
    y = corners[:, 1]

    x_min, x_max = np.min(x), np.max(x)
    y_min, y_max = np.min(y), np.max(y)

    x_tiles = int(
        np.ceil((x_max - x_min) / tile_size[0])
    )  # Number of tiles in the x direction
    y_tiles = int(
        np.ceil((y_max - y_min) / tile_size[1])
    )  # Number of tiles in the y direction

    # Generate the corners of each tile
    tiles = []
    for i in range(x_tiles):
        for j in range(y_tiles):
            # Calculate the corners of each tile
            tile_corners = [
                [x_min + i * tile_size[0], y_min + j * tile_size[1]],  # bottom-left
                [
                    x_min + (i + 1) * tile_size[0],
                    y_min + j * tile_size[1],
                ],  # bottom-right
                [
                    x_min + (i + 1) * tile_size[0],
                    y_min + (j + 1) * tile_size[1],
                ],  # top-right
                [x_min + i * tile_size[0], y_min + (j + 1) * tile_size[1]],  # top-left
            ]
            tiles.append(np.array(tile_corners))

    return tiles
