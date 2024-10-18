from PIL import Image

# import piexif
from PIL.ExifTags import TAGS, GPSTAGS

a = 6378137.0
b = 6356752.314245
inv_f = 298.257223563
e_sq = 6.69437999014 * 1e-3
import math
import numpy as np


class GPS_point:
    def __init__(self, lat=None, lon=None, alt=None, rad=False) -> None:
        if rad:
            self.lon = math.radians(lon)
            self.lat = math.radians(lat)
        else:
            self.lon = lon
            self.lat = lat
        self.alt = alt

    def inrad(self):
        return math.radians(self.lat), math.radians(self.lon), self.alt


class converter:
    def __init__(self, ref_path):
        self.ref_geodetic = GPS_point(
            self.extract_gps_data(self.get_image_properties(ref_path)["GPSInfo"])
        )
        self.ref_ECEF = geodetic2ecef(self.ref_geodetic.inrad())

        # print(f"Ref point ECEF Coordinates: {ecef_coords}")

        pass

    def get_image_properties(self, image_path):
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

    # Extract and convert GPS coordinates from the EXIF GPSInfo
    def extract_gps_data(self, gps_info):
        lat_dms = gps_info["GPSLatitude"]
        lat_ref = gps_info["GPSLatitudeRef"]
        lon_dms = gps_info["GPSLongitude"]
        lon_ref = gps_info["GPSLongitudeRef"]
        altitude = gps_info["GPSAltitude"]

        lat_dd = self._dms_to_dd(lat_dms, lat_ref)  # Latitude in decimal degrees
        lon_dd = self._dms_to_dd(lon_dms, lon_ref)  # Longitude in decimal degrees

        return lat_dd, lon_dd, altitude

    # Function to convert degrees, minutes, seconds to decimal degrees
    def _dms_to_dd(self, dms_tuple, ref):
        degrees = dms_tuple[0]
        minutes = dms_tuple[1] / 60
        seconds = dms_tuple[2] / 3600
        decimal_degrees = degrees + minutes + seconds
        if ref in ["S", "W"]:
            decimal_degrees = -decimal_degrees
        return decimal_degrees


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

    return np.dot(R, (p - p_ref))


# testing
"""
Given:
    Ref point: lon, lat, h(ASML)
    Point p: lon, lat, h(ASML)
Get:
    Point p: x, y, z (NED), wrt Ref point is (0,0,0) NED
"""


rimage_path = "/workspaces/real2map/APPEZZAMENTO PICCOLO/DJI_20240607121127_0003_D.JPG"

conv = Converter(rimahe_path)
# sec_image_path = (
#     "/workspaces/real2map/APPEZZAMENTO PICCOLO/DJI_20240607121129_0004_D_point0.JPG"
# )

# rimage_properties = get_image_properties(rimage_path)
# # print(rimage_properties["GPSInfo"])

# lat_dd, lon_dd, altitude = extract_gps_data(rimage_properties["GPSInfo"])
# # print(f"Latitude: {lat_dd}°, Longitude: {lon_dd}°, Altitude: {altitude} m")

# # Convert geodetic to ECEF
# ecef_coords = np.array(
#     geodetic2ecef(math.radians(lat_dd), math.radians(lon_dd), altitude)
# )  # .reshape((3, 1))
# print(f"Ref point ECEF Coordinates: {ecef_coords}")


# simage_properties = get_image_properties(sec_image_path)
# # print(simage_properties["GPSInfo"])

# slat_dd, slon_dd, saltitude = extract_gps_data(simage_properties["GPSInfo"])
# # print(f"second Latitude: {slat_dd}°, Longitude: {slon_dd}°, Altitude: {saltitude} m")

# # # Convert geodetic to ECEF
# secef_coords = np.array(
#     geodetic2ecef(math.radians(slat_dd), math.radians(slon_dd), saltitude)
# )  # .reshape((3, 1))
# print(f"2nd point ECEF Coordinates: {secef_coords}")

# # Obtain NED of s point given that ref point is (0,0) of NED
# r_ned = ecef2ned(ecef_coords, ecef_coords, lon_dd, lat_dd)
# print(r_ned)

# s_ned = ecef2ned(secef_coords, ecef_coords, lon_dd, lat_dd)
# print(s_ned)
