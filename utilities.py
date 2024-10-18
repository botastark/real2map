from PIL.ExifTags import TAGS, GPSTAGS
from PIL import Image
import libxmp
from matplotlib import pyplot as plt
import pyproj
import navpy
import numpy as np
from libxmp import XMPFiles, consts
import xml.etree.ElementTree as ET

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


def extract_gps_data(gps_info):
    lat_dms = gps_info["GPSLatitude"]
    lat_ref = gps_info["GPSLatitudeRef"]
    lon_dms = gps_info["GPSLongitude"]
    lon_ref = gps_info["GPSLongitudeRef"]
    altitude = gps_info["GPSAltitude"]
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

    if exif_data:
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
        # if "XMP Other" in exif_info:
        #     print("additional info found")
        #     other_info = {}
        #     for key in exif_info["XMP Other"].keys():
        #         gps_tag = GPSTAGS.get(key, key)
        #         gps_info[gps_tag] = exif_info["GPSInfo"][key]
        #     image_data["GPSInfo"] = gps_info

    return image_data


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


from libxmp import *


def extract_xmp(image_path):
    img = Image.open(image_path)

    metadata = {}
    xmp_data = img.getxmp()["xmpmeta"]["RDF"]["Description"]
    for key, value in xmp_data.items():
        metadata[key] = value

    return metadata
