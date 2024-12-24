from PIL import Image, ImageDraw
from PIL.ExifTags import TAGS
# from pyproj import Transformer
from ggmap_image import read_meta
from matching import match_roads, scale_for_matching, red_point_sz
from read_exif import gps_exift
from ggmap_image import find_pixel_coordinates
from match_class import DetectMatchingMap

import math
import cv2
import numpy as np


def rotate_image(img, angle, pts, scale_factor):
    if img.mode != "RGBA":
        img = img.convert("RGBA")

    center = (img.width / 2, img.height / 2)

    rotated_points = [rotate_point_by_center(p, center, - angle) for p in pts]
    ls1 = [rotate_point_by_center(p, center, - angle) for p in
           [(0, 0), (img.width, 0), (img.width, img.height), (0, img.height)]]

    rotated_img = img.rotate(angle, expand=True)
    # rotated_img = rotated_img.convert("RGBA")

    transparent_background = Image.new("RGBA", rotated_img.size, (0, 0, 0, 0))
    alpha_channel = rotated_img.split()[-1]  # Lấy kênh alpha
    transparent_background.paste(rotated_img, (0, 0), alpha_channel)

    rotated_img = transparent_background

    # Tính toán offset để điểm xoay phù hợp với kích thước ảnh đã xoay
    offset_x = (rotated_img.width - img.width) / 2
    offset_y = (rotated_img.height - img.height) / 2

    adjusted_points = [(x + offset_x, y + offset_y) for x, y in rotated_points]
    ls2 = [((x + offset_x) * scale_factor, (y + offset_y) * scale_factor) for x, y in ls1]

    # print(adjusted_points)

    new_width = int(rotated_img.width * scale_factor)
    new_height = int(rotated_img.height * scale_factor)

    scaled_img = rotated_img.resize((new_width, new_height), Image.Resampling.LANCZOS)

    sz = red_point_sz * scale_factor * 5

    pts = []
    draw = ImageDraw.Draw(scaled_img)
    for point in adjusted_points:
        x, y = point

        x *= scale_factor
        y *= scale_factor

        pts += [(x, y)]

        draw.ellipse((x - sz, y - sz, x + sz, y + sz), fill="red", outline="red")  # Vẽ điểm đỏ

    # Lưu ảnh tạm
    scaled_img.save('temp/img_2.png', format="PNG")
    # print(f"Ảnh đã được lưu tại: {output_path}")

    return 'temp/img_2.png', ls2, pts


def calculate_pixel_size(dpi):
    return 1 / dpi


def calculate_ground_resolution(pixel_size, altitude, focal_length):
    altitude_mm = altitude * 1000
    return (pixel_size * altitude_mm) / focal_length


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


def calc_real_dimension_with_drone_image(img_path, realsize_on_pixel_on_map, pts):
    img = Image.open(img_path)
    exiftinfor = gps_exift(img_path)

    scale_factor = scale_for_matching * exiftinfor.meter_per_pixel / realsize_on_pixel_on_map
    return rotate_image(img, exiftinfor.yaw, pts, scale_factor), (exiftinfor.longitude, exiftinfor.latitude, exiftinfor.crs)


def draw_bound(image, pts):
    points = np.array(pts, dtype=np.int32)
    print(points)
    # Định dạng mảng điểm cho OpenCV
    points = points.reshape((-1, 1, 2))

    # Vẽ đường đa giác kín (isClosed=True)
    cv2.polylines(image, [points], isClosed=True, color=(0, 255, 0), thickness=1)


def extract_and_draw_final(map_path, template_path, ref_points):
    gps = gps_exift(map_path)
    par1, dest_location = calc_real_dimension_with_drone_image(template_path, gps.meter_per_pixel, ref_points)

    _, rect, pts = par1
    target_lon, target_lat, meter_per_pixel = dest_location

    rotated_template = cv2.imread(par1[0])

    x, y, w, h = find_pixel_coordinates(gps.longitude, gps.latitude, gps.crs,
                                        target_lon, target_lat, dest_location[2],
                                        gps.meter_per_pixel, rotated_template.shape)

    ggmap_image = cv2.imread(map_path, cv2.IMREAD_UNCHANGED)
    scaled_ggmap_image = cv2.resize(ggmap_image, None, fx=5, fy=5, interpolation=cv2.INTER_LANCZOS4)
    cv2.imwrite('temp/scale.jpg', scaled_ggmap_image)

    match = DetectMatchingMap('temp/scale.jpg', 'temp/img_2.png', [x, y, w, h])
    nx, ny = match.result

    # best_match_temp = match.best_template
    temp_img = cv2.imread('temp/img_2.png')
    #
    best_angle = match.best_angle
    #
    center = (x + temp_img.shape[1] / 2, y + temp_img.shape[0] / 2)

    rotated_points = [rotate_point_by_center(p, center, best_angle) for p in pts]

    nx += x
    ny += y

    sc = match.best_scale / 5

    res = []
    for p in rotated_points:
        np = (int(nx + p[0] * sc), int(ny + p[1] * sc))
        res += [np]
        cv2.circle(ggmap_image, np, 2, (0, 0, 255), -1)

    draw_bound(ggmap_image, [(p[0] * sc + nx, p[1] * sc + ny) for p in rect])

    cv2.imwrite('temp/final.jpg', ggmap_image)
    cv2.imshow('Result', ggmap_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return res


if __name__ == '__main__':
    pts = extract_and_draw_final('1.tif', 'image5.jpg',
                                 ([4315, 1779], [2470, 2997], [3927, 1479], [1925, 2128], [2280, 2659]))
    print(pts)
