from PIL import Image

# import piexif
from PIL.ExifTags import TAGS, GPSTAGS


def get_image_properties(image_path):
    # Open the image file
    img = Image.open(image_path)

    # Initialize dictionary to store properties
    image_data = {}

    # Get basic image properties
    image_data["Format"] = img.format
    image_data["Mode"] = img.mode
    image_data["Size"] = img.size  # (width, height)
    image_data["Info"] = img.info

    # Extract EXIF data if present
    exif_data = img._getexif()

    if exif_data:
        # Initialize a dictionary for EXIF data
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


a = 6378137.0
b = 6356752.314245
inv_f = 298.257223563
e_sq = 6.69437999014 * 1e-3
import math
import numpy as np


def radius_n(lat):
    cot = math.cos(lat) / math.sin(lat)
    return a / (1 - e_sq / (1 + cot * cot)) ** 0.5


def geodetic2ecef(lat, lon, h):
    N = radius_n(lat)
    X = (N + h) * math.cos(lat) * math.cos(lon)
    Y = (N + h) * math.cos(lat) * math.sin(lon)
    Z = ((1 - e_sq) * N + h) * math.sin(lat)
    return (X, Y, Z)


def ecef2ned(p, p_ref, lon_ref, lat_ref):
    sin_lat = math.sin(lat_ref)
    cos_lat = math.cos(lat_ref)

    sin_lon = math.sin(lon_ref)
    cos_lon = math.cos(lon_ref)

    R = np.array(
        [
            [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
            [-sin_lon, cos_lon, 0],
            [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
        ]
    )
    assert p.shape == p_ref.shape

    return R * (p - p_ref)


# Function to convert degrees, minutes, seconds to decimal degrees
def dms_to_dd(dms_tuple, ref):
    degrees = dms_tuple[0]
    minutes = dms_tuple[1] / 60
    seconds = dms_tuple[2] / 3600
    decimal_degrees = degrees + minutes + seconds
    if ref in ["S", "W"]:
        decimal_degrees = -decimal_degrees
    return decimal_degrees


# Extract and convert GPS coordinates from the EXIF GPSInfo
def extract_gps_data(gps_info):
    lat_dms = gps_info["GPSLatitude"]
    lat_ref = gps_info["GPSLatitudeRef"]
    lon_dms = gps_info["GPSLongitude"]
    lon_ref = gps_info["GPSLongitudeRef"]
    altitude = gps_info["GPSAltitude"]

    lat_dd = dms_to_dd(lat_dms, lat_ref)  # Latitude in decimal degrees
    lon_dd = dms_to_dd(lon_dms, lon_ref)  # Longitude in decimal degrees

    return lat_dd, lon_dd, altitude


# testing
"""
Given:
    Ref point: lon, lat, h(ASML)
    Point p: lon, lat, h(ASML)
Get:
    Point p: x, y, z (NED), wrt Ref point is (0,0,0) NED
"""

ref_point = np.array([])

# Example usage
rimage_path = "/workspaces/real2map/APPEZZAMENTO PICCOLO/DJI_20240607121127_0003_D.JPG"
sec_image_path = (
    "/workspaces/real2map/APPEZZAMENTO PICCOLO/DJI_20240607121129_0004_D_point0.JPG"
)

rimage_properties = get_image_properties(rimage_path)
print(rimage_properties["GPSInfo"])

lat_dd, lon_dd, altitude = extract_gps_data(rimage_properties["GPSInfo"])
print(f"Latitude: {lat_dd}°, Longitude: {lon_dd}°, Altitude: {altitude} m")

# Convert geodetic to ECEF
ecef_coords = geodetic2ecef(math.radians(lat_dd), math.radians(lon_dd), altitude)
print(f"Ref point ECEF Coordinates: {ecef_coords}")


simage_properties = get_image_properties(rimage_path)
print(simage_properties["GPSInfo"])

slat_dd, slon_dd, saltitude = extract_gps_data(simage_properties["GPSInfo"])
print(f"second Latitude: {slat_dd}°, Longitude: {slon_dd}°, Altitude: {saltitude} m")

# Convert geodetic to ECEF
secef_coords = geodetic2ecef(math.radians(slat_dd), math.radians(slon_dd), saltitude)
print(f"2nd point ECEF Coordinates: {secef_coords}")
