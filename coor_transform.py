import math

import numpy as np
from pyproj import Transformer

from drone_image import rotate_point_by_center
from read_exif import gps_exift


class CoordTransform:
    def __init__(self, src_geo_exif: gps_exift, src_crs: str, dst_geo_exif: gps_exift, dst_crs: str):
        self.src_geo_exif = src_geo_exif
        self.src_crs = src_crs
        self.dst_geo_exif = dst_geo_exif
        self.dst_crs = dst_crs

    def convert_gps_pixel_to_geo(self, x, y) -> (float, float):
        yaw = self.dst_geo_exif.yaw

        # yaw_rad = math.radians(yaw)

        from_meter_to_geo = Transformer.from_crs(self.src_crs, self.dst_crs)
        from_geo_to_meter = Transformer.from_crs(self.dst_crs, self.src_crs)

        tl_x, tl_y = from_geo_to_meter.transform(self.dst_geo_exif.longitude, self.dst_geo_exif.latitude)

        rotated_x, rotated_y = rotate_point_by_center((x, y),
                                                      (self.dst_geo_exif.width // 2, self.dst_geo_exif.height // 2),
                                                      - yaw)

        X_prime = tl_x + self.dst_geo_exif.meter_per_pixel * (rotated_x - self.dst_geo_exif.width // 2)
        Y_prime = tl_y - self.dst_geo_exif.meter_per_pixel * (rotated_y - self.dst_geo_exif.height // 2)

        # Kết quả
        print(f"Tọa độ địa lý: X = {X_prime}, Y = {Y_prime}")

        return from_meter_to_geo.transform(X_prime, Y_prime)

    def convert_geo_to_tif_pixel(self, lon, lat) -> (int, int):
        from_geo_to_meter = Transformer.from_crs(self.dst_crs, self.src_crs)

        tl_lon, tl_lat = from_geo_to_meter.transform(self.src_geo_exif.longitude, self.src_geo_exif.latitude)

        col = (lon - tl_lon) // self.src_geo_exif.meter_per_pixel
        row = (tl_lat - lat) // self.src_geo_exif.meter_per_pixel

        return int(col), int(row)
