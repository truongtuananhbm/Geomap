from PIL import Image, ImageDraw
import math
from PIL.ExifTags import TAGS
from geopy.distance import geodesic
from pyproj import Transformer

from matching import scale_for_matching
from read_exif import gps_exift
from geopy.distance import geodesic


# Hàm tính khoảng cách geodesic giữa hai điểm
def haversine(lat1, lon1, lat2, lon2):
    return geodesic((lat1, lon1), (lat2, lon2)).kilometers * 0.621371


def read_meta(image):
    exif_data = image.getexif()

    if exif_data is not None:
        meta_data = {TAGS.get(tag_id, tag_id): value for tag_id, value in exif_data.items()}
        meta_data['Width'] = image.width
        meta_data['Height'] = image.height

        for tag_id, value in meta_data.items():
            tag_name = TAGS.get(tag_id, tag_id)
            # print(f"{tag_name}: {value}")

        return meta_data
    else:
        print("Không tìm thấy metadata trong ảnh.")


# def read_gg_map(origin_lon,origin_lat, target_lon, target_lat):
#     img = Image.open('1.tif')
#     read_meta(img)

#     # Các tham số đầu vào
#     image_width = 856  # Chiều rộng ảnh (pixel)
#     image_height = 496  # Chiều cao ảnh (pixel)
#     meter_per_pixel = 4.77  # Mỗi pixel tương ứng với 4.77 mét

#     print(f"Tọa độ chuyển sang EPSG:4326 - Vĩ độ: {origin_lon}, Kinh độ: {origin_lat}")

#     # Tọa độ đích (long, lat)
#     target_lon = 37 + 24/60 + 19.26/3600  # 37 deg 24' 19.26" E
#     target_lat = 54 + 50/60 + 22.29/3600  # 54 deg 50' 22.29" N

#     # Gọi hàm tìm tọa độ pixel
#     find_pixel_coordinates(origin_lon, origin_lat, target_lon, target_lat, meter_per_pixel)


# Hàm chuyển đổi từ mét sang pixel
def distance_to_pixel(distance_m, meter_per_pixel):
    return distance_m / meter_per_pixel


# Từ tọa độ của 2 ảnh tính ra vị trí của ảnh 2 trên ảnh 1
def target_location(origin_lon, origin_lat, org_crs, target_lon, target_lat, target_crs, origin_meter_per_pixel):
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857", always_xy=True)

    # Chuyển tọa độ qua mét nếu đang là tọa độ địa lý
    if org_crs == "epsg:4326":
        origin_lon, origin_lat = transformer.transform(origin_lon, origin_lat)

    if target_crs == "epsg:4326":
        target_lon, target_lat = transformer.transform(target_lon, target_lat)

    # Trả về tọa độ pixel
    return int((target_lon - origin_lon) / origin_meter_per_pixel), int(
        (origin_lat - target_lat) / origin_meter_per_pixel)


def compute_roi_size(target_shape):
    return 2 * target_shape[0] / scale_for_matching, 2 * target_shape[1] / scale_for_matching


# Hàm xác định tọa độ pixel (x, y)
def find_pixel_coordinates(origin_lon, origin_lat, org_crs, target_lon, target_lat, target_crs, origin_meter_per_pixel,
                           target_shape):
    # Tính khoảng cách giữa tọa độ gốc và tọa độ đích
    # distance_km = haversine(origin_lat, origin_lon, target_lat, target_lon)
    # print('Long Lat:', origin_lon, origin_lat, target_lon, target_lat, distance_km)
    # Chuyển đổi từ km sang mét
    # distance_m = distance_km * 1000

    view_size = compute_roi_size(target_shape)

    view_width = int(view_size[0])
    view_height = int(view_size[1])

    # Tính toán khoảng cách tương ứng với pixel
    # pixel_distance = distance_to_pixel(distance_m, origin_meter_per_pixel)

    # Tính toán x và y trên ảnh, giả sử gốc tọa độ (0, 0) nằm ở góc trên bên trái
    # Ta giả định rằng sự dịch chuyển dọc theo kinh độ sẽ ảnh hưởng đến chiều ngang (x),
    # và sự dịch chuyển dọc theo vĩ độ sẽ ảnh hưởng đến chiều dọc (y).

    # Tính toán tọa độ pixel theo kinh độ (x)
    # delta_lon = target_lon - origin_lon
    # pixel_y = - delta_lon * pixel_distance * 100
    #
    # # Tính toán tọa độ pixel theo vĩ độ (y)
    # delta_lat = target_lat - origin_lat
    # pixel_x = (delta_lat * pixel_distance) * 100

    pixel_x, pixel_y = target_location(origin_lon, origin_lat, org_crs, target_lon, target_lat, target_crs,
                                       origin_meter_per_pixel)

    # In kết quả
    print(f"Tọa độ pixel trên ảnh (x, y) cho tọa độ đích ({target_lon}, {target_lat}) là:")
    print(f"X: {pixel_x:.2f} pixels")
    print(f"Y: {pixel_y:.2f} pixels")

    return int(pixel_x - view_width / 2), int(pixel_y - view_height / 2), view_width, view_height
