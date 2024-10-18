a = 6378137.0
b = 6356752.314245
inv_f = 298.257223563
e_sq = 6.69437999014 * 1e-3
import math
import numpy as np
import pygeodesy
from pygeodesy.ellipsoidalKarney import LatLon

ginterpolator = pygeodesy.GeoidKarney("./geoids/egm84-15.pgm")


class GPS_point:
    def __init__(self, data, rad=False) -> None:
        lat, lon, alt = data
        self.rad = rad
        if rad:
            self.lon = math.radians(lon)
            self.lat = math.radians(lat)
        else:
            self.lon = float(lon)
            self.lat = float(lat)
        self.alt = float(alt)

    def inrad(self):
        if self.rad == False:
            return GPS_point(
                (math.radians(self.lat), math.radians(self.lon), self.alt), rad=True
            )
        else:
            return GPS_point((self.lat, self.lon, self.alt))

    def __str__(self):
        if self.rad == True:
            return f"lat: {self.lat}rad, lon: {self.lon}rad, alt: {self.alt}m)"
        return f"lat: {self.lat}°, lon: {self.lon}°, alt: {self.alt}m"


class ECEF_point:
    def __init__(self, data) -> None:
        if data is None:
            data = (0, 0, 0)
        self.X, self.Y, self.Z = data

    def __str__(self):
        return f"ECEF_point(X: {self.X}, Y: {self.Y}, Z: {self.Z})"

    def __repr__(self):
        return f"ECEF_point({self.X}, {self.Y}, {self.Z})"

    # Method to subtract two ECEF_point objects
    def __sub__(self, other):
        if isinstance(other, ECEF_point):
            # Subtract the corresponding attributes and return a new ECEF_point
            return ECEF_point((self.X - other.X, self.Y - other.Y, self.Z - other.Z))
        else:
            raise TypeError("Subtraction is only supported between ECEF_point objects.")

    def to_array(self):
        return np.array([self.X, self.Y, self.Z])


class WRF_point:
    def __init__(self, data, frame="NED"):
        self.x, self.y, self.z = data
        self.frame = frame

    def __str__(self):
        return f"WRF_point(x: {self.x}, y: {self.y}, z: {self.z}, frame: {self.frame})"

    def __repr__(self):
        return f"WRF_point({self.x}, {self.y}, {self.z}, frame={self.frame})"

    # Function to convert from NED to ENU
    def to_ENU(self):
        if self.frame == "NED":
            enu_x = self.y  # East (from Y in NED)
            enu_y = self.x  # North (from X in NED)
            enu_z = -self.z  # Up (reverse of Z in NED)
            return WRF_point((enu_x, enu_y, enu_z), frame="ENU")
        else:
            print("Already in ENU frame.")
            return self

    # Function to convert from ENU to NED
    def to_NED(self):
        if self.frame == "ENU":
            ned_x = self.y  # North (from Y in ENU)
            ned_y = self.x  # East (from X in ENU)
            ned_z = -self.z  # Down (reverse of Z in ENU)
            return WRF_point(ned_x, ned_y, ned_z)
        else:
            print("Already in NED frame.")
            return self


class Converter:
    def __init__(self, gps_info):
        self.ref_geodetic = self.extract_gps_data(gps_info)
        self.ref_ECEF = self.geodetic2ecef(self.ref_geodetic.inrad())
        # Get the geoid height
        single_position = LatLon(self.ref_geodetic.lat, self.ref_geodetic.lon)
        h = ginterpolator(single_position)
        print(h)

    def extract_gps_data(self, gps_info):

        lat_dms = gps_info["GPSLatitude"]
        lat_ref = gps_info["GPSLatitudeRef"]
        lon_dms = gps_info["GPSLongitude"]
        lon_ref = gps_info["GPSLongitudeRef"]
        altitude = gps_info["GPSAltitude"]

        lat_dd = self._dms_to_dd(lat_dms, lat_ref)  # Latitude in decimal degrees
        lon_dd = self._dms_to_dd(lon_dms, lon_ref)  # Longitude in decimal degrees

        return GPS_point((lat_dd, lon_dd, altitude))

    # Function to convert degrees, minutes, seconds to decimal degrees
    def _dms_to_dd(self, dms_tuple, ref):
        degrees = dms_tuple[0]
        minutes = dms_tuple[1] / 60
        seconds = dms_tuple[2] / 3600
        decimal_degrees = degrees + minutes + seconds
        if ref in ["S", "W"]:
            decimal_degrees = -decimal_degrees
        return decimal_degrees

    def radius_n(self, lat):
        return a / math.sqrt(1 - e_sq * math.sin(lat) ** 2)

    def geodetic2ecef(self, point):
        N = self.radius_n(point.lat)
        out = ECEF_point(None)
        out.X = (N + point.alt) * math.cos(point.lat) * math.cos(point.lon)
        out.Y = (N + point.alt) * math.cos(point.lat) * math.sin(point.lon)
        out.Z = ((1 - e_sq) * N + point.alt) * math.sin(point.lat)
        return out

    def ecef2ned(self, p):
        sin_lat = math.sin(self.ref_geodetic.lat)
        cos_lat = math.cos(self.ref_geodetic.lat)

        sin_lon = math.sin(self.ref_geodetic.lon)
        cos_lon = math.cos(self.ref_geodetic.lon)

        R = np.array(
            [
                [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
                [-sin_lon, cos_lon, 0],
                [-cos_lat * cos_lon, -cos_lat * sin_lon, -sin_lat],
            ]
        )
        return WRF_point(np.dot(R, (p - self.ref_ECEF).to_array()))
