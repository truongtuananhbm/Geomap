import subprocess
import re
import math
from pyproj import Proj, Transformer


class gps_exift:
    def dms_to_decimal(self, dms_str):
        m1 = re.search(r'(\d+) deg (\d+)\' (\d+\.\d+)\" N', dms_str)
        m2 = re.search(r'(\d+) deg (\d+)\' (\d+\.\d+)\" E', dms_str)

        # Chuyển đổi vĩ độ
        deg1 = int(m1.group(1))
        min1 = int(m1.group(2))
        sec1 = float(m1.group(3))

        # Chuyển đổi kinh độ
        deg2 = int(m2.group(1))
        min2 = int(m2.group(2))
        sec2 = float(m2.group(3))

        # Tính toán Decimal Degrees cho vĩ độ và kinh độ
        decimal1 = deg1 + (min1 / 60) + (sec1 / 3600)
        decimal2 = deg2 + (min2 / 60) + (sec2 / 3600)

        return decimal2, decimal1

    def __init__(self, file):
        # Lệnh cần chạy
        command = ['exiftool.exe', file]

        # Chạy lệnh và lấy kết quả đầu ra
        result = subprocess.run(command, capture_output=True, text=True)

        # In kết quả từ stdout (standard output)
        output = result.stdout
        #
        # print(output)

        for s in output.split('\n'):
            key = s.split(':')[0]
            value = s.split(':')[-1].strip()

            if 'Field Of View' in key:
                self.fov = float(value.replace('deg', ''))
            elif 'GPS Position' in key:
                self.longitude, self.latitude = self.dms_to_decimal(value)
                self.crs = "epsg:4326"
            elif 'Yaw' in key:
                self.yaw = - float(value)
            elif 'Pitch' in key:
                self.pitch = float(value)
            elif 'Roll' in key:
                self.roll = float(value)

            elif 'GPS Altitude' in key:
                self.altitude = float(value.replace('m', ''))
            elif 'Image Width' in key:
                self.width = float(value)
            elif 'Image Height' in key:
                self.height = float(value)
            elif 'X Resolution' in key:
                self.dpi = float(value)
            elif 'Pixel Scale' in key:
                # print('px', value.split(' ')[0])
                self.meter_per_pixel = float(value.split(' ')[0])
            elif 'Model Tie Point' in key:
                x = float(value.split(' ')[3])
                y = float(value.split(' ')[4])

                # transformer = Transformer.from_crs("epsg:3857", "epsg:4326")
                self.longitude, self.latitude = x, y

                self.crs = "epsg:3857"

        if hasattr(self, 'altitude') and hasattr(self, 'width') and hasattr(self, 'fov') and (
                not hasattr(self, 'meter_per_pixel')):
            self.meter_per_pixel = 2 * self.altitude * math.tan(math.radians(self.fov / 2)) / self.width

    def __str__(self):
        # (fov={self.fov}, gps={self.longitude, self.latitude}, yaw={self.yaw}, altitude={self.altitude}, width={self.width}, height={self.height}, dpi={self.dpi})
        return f"GPS={self.longitude, self.latitude}"


if __name__ == '__main__':
    cls = gps_exift('1.tif')
    print(cls)
