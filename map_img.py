from PIL import Image, ImageDraw
from PIL.ExifTags import TAGS
from pyproj import Transformer
import piexif
import math

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

def calculate_real_pixel_size(fov, altitude, image_width):
    # Độ cao của drone (m)
    altitude_m = altitude

    # Tính chiều rộng thực tế của khu vực mà camera nhìn thấy
    real_width = 2 * altitude_m * math.tan(math.radians(fov / 2))

    # Kích thước thực tế của mỗi pixel
    pixel_size = real_width / image_width

    return pixel_size


def transform_location_pos():
  # Đầu vào
    pixel_size = 4.777314267823555
    tie_point_x, tie_point_y = 4162108.457, 7331784.814
    longitude, latitude = 37.4713397026062, 54.792112458942945

    # Chuyển đổi từ EPSG:4326 sang EPSG:3857
    transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")
    x, y = transformer.transform(latitude, longitude)

    print('xy',x,y)

    # Tính toán pixel column và row
    pixel_column = (x - tie_point_x) / pixel_size
    pixel_row = (tie_point_y - y) / pixel_size

    print(f"Pixel Column: {int(pixel_column)}, Pixel Row: {int(pixel_row)}")

# def get_gps_info(image_path):
#     """
#     Đọc thông tin GPS từ file ảnh.
    
#     Args:
#         image_path (str): Đường dẫn đến file ảnh.
    
#     Returns:
#         dict: Thông tin GPS hoặc thông báo lỗi nếu không tìm thấy.
#     """
#     try:
#         # Load Exif data từ ảnh
#         img = Image.open(image_path)
#         exif_data = piexif.load(img.info['exif'])
        
#         # Lấy thông tin GPS
#         gps_data = exif_data.get("GPS")
#         if not gps_data:
#             return {"error": "Không tìm thấy thông tin GPS trong ảnh."}
        
#         # Chuyển đổi tọa độ từ định dạng thô sang định dạng dễ hiểu
#         gps_info = {}
#         for tag, value in gps_data.items():
#             tag_name = piexif.GPSIFD.get(tag, tag)
#             gps_info[tag_name] = value
        
#         return gps_info
#     except Exception as e:
#         return {"error": str(e)}
    
def convert_to_decimal(dms, ref):
    """
    Chuyển đổi tọa độ từ độ-phút-giây (DMS) sang dạng thập phân.
    
    Args:
        dms (tuple): Tọa độ DMS dưới dạng ((độ, mẫu), (phút, mẫu), (giây, mẫu)).
        ref (str): Hướng ('N', 'S', 'E', 'W').
    
    Returns:
        float: Tọa độ thập phân.
    """
    degree = dms[0][0] / dms[0][1]
    minute = dms[1][0] / dms[1][1]
    second = dms[2][0] / dms[2][1]
    decimal = degree + (minute / 60.0) + (second / 3600.0)
    if ref in ['S', 'W']:
        decimal = -decimal
    return decimal

    
def get_gps_info(img):
    """
    Đọc thông tin GPS từ file ảnh.
    
    Args:
        image_path (str): Đường dẫn đến file ảnh.
    
    Returns:
        dict: Thông tin GPS hoặc thông báo lỗi nếu không tìm thấy.
    """

    # Load Exif data từ ảnh
    # img = Image.open(image_path)
    exif_data = piexif.load(img.info.get('exif', b""))
    
    # Lấy thông tin GPS
    gps_data = exif_data.get("GPS")
    if not gps_data:
        return {"error": "Không tìm thấy thông tin GPS trong ảnh."}
    
    # Xử lý dữ liệu GPS
    gps_info = {}
    for tag, value in gps_data.items():
        gps_info[tag] = value
    
    return gps_info


def latlon_to_webmercator(lat, lon):
    R = 6378137  # bán kính trái đất trong mét
    x = lon * math.pi / 180 * R
    y = math.log(math.tan((90 + lat) * math.pi / 360)) * R
    return x, y

def rotate_image(img, angle, pts, scale_factor):
    img= img.convert('RGBA')

    center = (img.width / 2, img.height / 2)
    rotated_points = [rotate_point_by_center(p, center, - angle) for p in pts]
    ls1 = [rotate_point_by_center(p, center, - angle) for p in [(0,0),(img.width,0),(img.width,img.height),(0,img.height)]]
    
    rotated_img = img.rotate(angle, expand=True)
    rotated_img = rotated_img.convert("RGBA")

    # Tính toán offset để điểm xoay phù hợp với kích thước ảnh đã xoay
    offset_x = (rotated_img.width - img.width) / 2
    offset_y = (rotated_img.height - img.height) / 2

    adjusted_points = [(x + offset_x, y + offset_y) for x, y in rotated_points]
    print(adjusted_points)

    new_width = int(rotated_img.width * scale_factor)
    new_height = int(rotated_img.height * scale_factor)

    scaled_img = rotated_img.resize((new_width, new_height), Image.Resampling.LANCZOS)

    rect = [(x * scale_factor, y * scale_factor) for x, y in ls1]

    sz = 5 * scale_factor
    # Vẽ điểm lên ảnh xoay
    draw = ImageDraw.Draw(scaled_img)
    res = []

    print(offset_x, offset_y)

    for x,y in adjusted_points:
        x *= scale_factor
        y *= scale_factor

        res += [[offset_x + x, offset_y + y]]
        draw.ellipse((x - sz, y - sz, x + sz, y + sz), fill="red", outline="red")  # Vẽ điểm đỏ

    output_path = 'temp/img_2.png'
    scaled_img.save(output_path, format="PNG")
    print(f"Ảnh đã được lưu tại: {output_path}")
    return output_path, res, rect

import math

def calculate_pixel_size(dpi):
    return 1 / dpi

def calculate_ground_resolution(pixel_size, altitude, focal_length):
    altitude_mm = altitude * 1000
    return (pixel_size * altitude_mm) / focal_length

# def calculate_fov(image_width, pixel_size, focal_length):
#     image_width_mm = image_width * pixel_size
#     fov = 2 * math.atan((image_width_mm / 2) / focal_length)

#     return math.degrees(fov)

def rotate_point_by_center(point, center, angle):
    # Chuyển đổi góc từ độ sang radian
    rad = math.radians(angle)
    cos_angle = math.cos(rad)
    sin_angle = math.sin(rad)

    # Tâm xoay
    cx, cy = center

    x, y = point
    # Tịnh tiến điểm về tâm
    tx, ty = x - cx, y - cy
    
    # Xoay điểm
    new_x = tx * cos_angle - ty * sin_angle
    new_y = tx * sin_angle + ty * cos_angle

    # Dịch ngược điểm về vị trí ban đầu
    rotated_x = new_x + cx
    rotated_y = new_y + cy

    return (rotated_x, rotated_y)


#Tính k1ich thước thực từ hình gg
def calc_real_dimension_with_gg_image(img_path):
    return None

def pixel_to_latlon(points, center_lon, center_lat, image_width, focal_length, angle_deg, fov):
    # Chuyển góc từ độ sang radian
    # angle_rad = math.radians(angle_deg)
    
    # Bán kính Trái đất (m)
    EARTH_RADIUS = 6371000
    focal_length = 20.0
    # Danh sách kết quả
    geo_points = []
    
    for x, y in points:
        # Tính khoảng cách thực tế (m) từ tâm ảnh
        # Công thức: khoảng cách = (pixel vị trí / chiều rộng pixel) * khoảng cách ngang thực tế
        distance_x = (x - image_width / 2) * (focal_length / math.tan(math.radians(fov / 2)))
        distance_y = y * (focal_length / math.tan(math.radians(fov / 2)))

        # Xoay điểm theo góc máy ảnh
        rotated_x = distance_x * math.cos(math.radians(angle_deg)) - distance_y * math.sin(math.radians(angle_deg))
        rotated_y = distance_x * math.sin(math.radians(angle_deg)) + distance_y * math.cos(math.radians(angle_deg))

        # Chuyển đổi sang tọa độ địa lý
        new_lat = center_lat + (rotated_y / EARTH_RADIUS) * (180 / math.pi)
        new_lon = center_lon + (rotated_x / (EARTH_RADIUS * math.cos(math.radians(center_lat)))) * (180 / math.pi)

        # Thêm vào danh sách kết quả
        geo_points.append((new_lon, new_lat))
        print(f"www.google.com/maps/?q={new_lat},{new_lon}")
        

    print(len(points))
    draw_geo_points(geo_points)

    return geo_points

def draw_geo_points(geo_points, output_path="output_canvas.jpg", canvas_size=(800, 800)):
    """
    Vẽ các điểm địa lý lên một canvas trắng.

    Args:
        geo_points (list): Danh sách các tọa độ địa lý dạng [(lon, lat), ...].
        output_path (str): Đường dẫn lưu file ảnh kết quả.
        canvas_size (tuple): Kích thước canvas (width, height).
    """
    # Tạo một canvas trắng
    width, height = canvas_size
    img = Image.new("RGB", (width, height), "white")
    draw = ImageDraw.Draw(img)
    
    # Chuyển đổi tọa độ địa lý sang tọa độ pixel trên canvas
    min_lon = min(point[0] for point in geo_points)
    max_lon = max(point[0] for point in geo_points)
    min_lat = min(point[1] for point in geo_points)
    max_lat = max(point[1] for point in geo_points)

    print(len(geo_points), geo_points, min_lon, max_lon, min_lat, max_lat)

    def geo_to_canvas(lon, lat):
        """Chuyển tọa độ địa lý sang tọa độ pixel trên canvas."""
        x = (lon - min_lon) / (max_lon - min_lon) * (width - 20) + 10  # Padding 10px
        y = (max_lat - lat) / (max_lat - min_lat) * (height - 20) + 10
        return int(x), int(y)
    
    # Vẽ từng điểm lên canvas
    for lon, lat in geo_points:
        x, y = geo_to_canvas(lon, lat)
        draw.ellipse((x - 5, y - 5, x + 5, y + 5), fill="red", outline="black")  # Vẽ điểm đỏ
    
    # Lưu ảnh kết quả
    img.save(output_path)
    print(f"Ảnh đã được lưu tại {output_path}")

#Tính k1ich thước thực hình drone
def calc_real_dimension_with_drone_image(img_path, pts):
    img = Image.open(img_path)

    infor = read_meta(img)
    gps = get_gps_info(img)

    # Các thông số đầu vào
    xdpi = infor['XResolution']  # DPI của ảnh
    ydpi = infor['YResolution']  # DPI của ảnh

    focal_length = 20  # Focal length của camera (mm)
    image_width = infor['Width']  # Chiều rộng ảnh (pixels)
    image_height = infor['Height']  # Chiều cao ảnh (pixels)

    
    latitude_dms = gps[2]
    longitude_dms = gps[4]

    

    latitude = convert_to_decimal(latitude_dms, 'N')
    longitude = convert_to_decimal(longitude_dms, 'E')
    
    print(f"www.google.com/maps/?q={latitude},{longitude}")
    print(f"Location: {longitude:.6f},{latitude:.6f}")

    attitude = gps[6][0] / gps[6][1]  # Độ cao bay (m)
    print(f"Attitude: {attitude:.6f} m")

    angle = 180 + gps[7][2][0] /gps[7][2][1]
    print(f"Angle: {angle:.2f}")
    

    pixel_size = calculate_pixel_size(xdpi)
    print(f"Pixel Size: {pixel_size:.6f} inch/pixel")

    ground_resolution = calculate_real_pixel_size(61.9,366.67,5456)
    print(f"Ground Resolution: {ground_resolution:.2f} m/pixel. Image real size is {ground_resolution * image_width:.2f} m * {ground_resolution * image_height:.2f} m")
    
    scale_factor = 5 * ground_resolution / 4.77
    return rotate_image(img,angle,pts, scale_factor)

from geopy.distance import geodesic

# Hàm tính khoảng cách geodesic giữa hai điểm
def geodesic_distance(lat1, lon1, lat2, lon2):
    # Khoảng cách theo Geodesic tính bằng km
    distance_km = geodesic((lat1, lon1), (lat2, lon2)).km
    return distance_km

# Hàm chuyển đổi từ mét sang pixel
def distance_to_pixel(distance_m, meter_per_pixel):
    return distance_m / meter_per_pixel

# Hàm xác định tọa độ pixel (x, y)
def find_pixel_coordinates(origin_lon, origin_lat, target_lon, target_lat, meter_per_pixel):
    # Tính khoảng cách giữa tọa độ gốc và tọa độ đích
    distance_km = geodesic_distance(origin_lat, origin_lon, target_lat, target_lon)
    
    # Chuyển đổi từ km sang mét
    distance_m = distance_km * 1000

    # Tính toán khoảng cách tương ứng với pixel
    pixel_distance = distance_to_pixel(distance_m, meter_per_pixel)
    
    # Tính toán x và y trên ảnh, giả sử gốc tọa độ (0, 0) nằm ở góc trên bên trái
    # Ta giả định rằng sự dịch chuyển dọc theo kinh độ sẽ ảnh hưởng đến chiều ngang (x),
    # và sự dịch chuyển dọc theo vĩ độ sẽ ảnh hưởng đến chiều dọc (y).

    # Tính toán tọa độ pixel theo kinh độ (x)
    delta_lon = target_lon - origin_lon
    pixel_x = delta_lon * pixel_distance
    
    # Tính toán tọa độ pixel theo vĩ độ (y)
    delta_lat = target_lat - origin_lat
    pixel_y = delta_lat * pixel_distance
    
    # In kết quả
    print(f"Tọa độ pixel trên ảnh (x, y) cho tọa độ đích ({target_lon}, {target_lat}) là:")
    print(f"X: {pixel_x:.2f} pixels")
    print(f"Y: {pixel_y:.2f} pixels")

def read_gg_map():
    img = Image.open('1.tif')
    read_meta(img)

    # Các tham số đầu vào
    image_width = 856  # Chiều rộng ảnh (pixel)
    image_height = 496  # Chiều cao ảnh (pixel)
    meter_per_pixel = 4.77  # Mỗi pixel tương ứng với 4.77 mét

    # Tọa độ gốc (long, lat)
    from pyproj import Proj, transform

    # Hệ tọa độ EPSG:3857 (Web Mercator)
    proj_3857 = Proj(init="epsg:3857")

    # Hệ tọa độ EPSG:4326 (WGS84)
    proj_4326 = Proj(init="epsg:4326")

    # Tọa độ EPSG:3857 (x, y) bạn đã có
    x = 4162108.457013141
    y = 7331784.814000026

    # Chuyển đổi sang EPSG:4326 (longitude, latitude)
    origin_lon, origin_lat = transform(proj_3857, proj_4326, x, y)

    print(f"Tọa độ chuyển sang EPSG:4326 - Vĩ độ: {origin_lon}, Kinh độ: {origin_lat}")

    # Tọa độ đích (long, lat)
    target_lon = 37 + 24/60 + 19.26/3600  # 37 deg 24' 19.26" E
    target_lat = 54 + 50/60 + 22.29/3600  # 54 deg 50' 22.29" N

    # Gọi hàm tìm tọa độ pixel
    find_pixel_coordinates(origin_lon, origin_lat, target_lon, target_lat, meter_per_pixel)


 
