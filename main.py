import cv2
import cv2 as cv
import os
import numpy as np
import math
import itertools
from copy import deepcopy
from pyproj import Transformer
import rasterio
from rasterio.transform import Affine
from drone_image import extract_and_draw_final, rotate_point_by_center
from read_exif import gps_exift


def convert_gps_pixel_to_geo(geo_exif, x, y) -> (float, float):
    yaw = geo_exif.yaw

    tl_x, tl_y = geo_exif.longitude, geo_exif.latitude
    from_meter_to_geo = None

    # yaw_rad = math.radians(yaw)

    # Nếu là kinh độ, vĩ độ là tọa độ địa lý thì chuyển qua tọa độ mét
    if geo_exif.crs == "epsg:4326":
        from_meter_to_geo = Transformer.from_crs("epsg:3857", geo_exif.crs)
        from_geo_to_meter = Transformer.from_crs(geo_exif.crs, "epsg:3857")

        tl_x, tl_y = from_geo_to_meter.transform(tl_x, tl_y)

    # Tính vị trí mới của điểm 0, 0 khi xoay (Theo pixel)
    tl_rotated_x, tl_rotated_y = rotate_point_by_center((0, 0),
                                                        (geo_exif.width // 2, geo_exif.height // 2),
                                                        - yaw)

    # Tính vị trí mới của x, y khi xoay (Theo pixel)
    rotated_x, rotated_y = rotate_point_by_center((x, y),
                                                  (geo_exif.width // 2, geo_exif.height // 2),
                                                  - yaw)

    # Chuyển vị trí x, y qua tọa độ mét
    X_prime = tl_x + geo_exif.meter_per_pixel * (rotated_x - tl_rotated_x)
    Y_prime = tl_y - geo_exif.meter_per_pixel * (rotated_y - tl_rotated_y)

    geo_x, geo_y = from_meter_to_geo.transform(X_prime, Y_prime)

    # Kết quả
    print(f"Tọa độ địa lý: X = {geo_x}, Y = {geo_y}")

    return geo_x, geo_y


def get_center(points_group, r=10, road_mask=None):
    center_points = []
    for point_1, point_2 in itertools.combinations(points_group, 2):
        center_position = (point_1 + point_2) // 2
        cx, cy = center_position[0], center_position[1]

        if cy - r > 0 and cy + r < road_mask.shape[0] and cx - r > 0 and cx + r < road_mask.shape[1]:
            if np.mean(road_mask[cy - r:cy + r, cx - r:cx + r]).item() > 250:
                center_points.append(center_position)
    return np.expand_dims(np.stack(center_points), 1) if center_points else False


def find_center(contour, batch_size=10, road_mask=None):
    contour_list = sorted(contour, key=lambda ele: (ele[0][1], ele[0][0]))
    sorted_contour = np.array(contour_list).squeeze(1)
    data_length = len(sorted_contour)

    center_list_master = [
        get_center(sorted_contour[start:start + batch_size], road_mask=road_mask)
        if start + batch_size <= data_length
        else get_center(sorted_contour[start:data_length], road_mask=road_mask)
        for start in range(0, data_length, batch_size)
    ]
    center_list_master = [ele for ele in center_list_master if ele is not False]
    return np.concatenate(center_list_master) if center_list_master else np.array([])


def find_intersections(total_center, road_mask, img, R=210):
    filter_centers = []
    for ith, [[cx, cy]] in enumerate(total_center):
        if cy - R > 0 and cy + R < road_mask.shape[0] and cx - R > 0 and cx + R < road_mask.shape[1]:
            region = cv.bitwise_not(road_mask[cy - R:cy + R, cx - R:cx + R])
            if region is not None:
                roi_contours, _ = cv.findContours(region, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
                roi_S = region.shape[0] * region.shape[1]
                roi_contours = [cnt for cnt in roi_contours if cv.contourArea(cnt) > roi_S * 0.04]

                if len(roi_contours) == 3:
                    sub_img = deepcopy(img[cy - R:cy + R, cx - R:cx + R])
                    cv.drawContours(sub_img, roi_contours, -1, (0, 255, 0), 10)
                    os.makedirs('temp/roi', exist_ok=True)
                    cv.imwrite(f'temp/roi/region_{ith}.jpg', sub_img)
                    filter_centers.append([[cx, cy]])
    return np.array(filter_centers)


def distance(p1, p2):
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def reduce_points(points, radius=30):
    reduced_points = []
    processed_points = set()

    for i, p1 in enumerate(points):
        if i in processed_points:
            continue
        reduced_points.append(p1)
        processed_points.add(i)

        for j in range(i + 1, len(points)):
            if distance(p1, points[j]) <= radius:
                processed_points.add(j)
    return reduced_points


def pixel_to_geo(x, y, transform):
    lon = transform.c + transform.a * x + transform.b * y
    lat = transform.f + transform.d * x + transform.e * y
    return lon, lat


def convert_coordinates(lon, lat, src_crs, target_crs="EPSG:4326"):
    transformer = Transformer.from_crs(src_crs, target_crs, always_xy=True)
    return transformer.transform(lon, lat)


def match_image(src_image_path, pts, dst_image_path, match_pts):
    src_image = cv.imread(src_image_path)
    dst_image = cv.imread(dst_image_path)

    dst_image_upscale = cv2.resize(dst_image, (dst_image.shape[1] * 3, dst_image.shape[0] * 3))

    src_scale = dst_image.shape[0] / src_image.shape[0]

    src_image_scale = cv2.resize(src_image, (int(src_image.shape[1] * src_scale), dst_image.shape[0]))

    final_image = np.zeros((dst_image_upscale.shape[0] + src_image_scale.shape[0], dst_image_upscale.shape[1], 3),
                           dtype=dst_image.dtype)

    pts = [[pnt[0] * src_scale, pnt[1] * src_scale] for pnt in
           pts]
    match_pts = [[pnt[0] * 3, pnt[1] * 3 + src_image_scale.shape[0]] for pnt in match_pts]

    final_image[:src_image_scale.shape[0], :src_image_scale.shape[1], :] = src_image_scale
    final_image[src_image_scale.shape[0]:, :, :] = dst_image_upscale

    for (pnt, match_pnt) in zip(pts, match_pts):
        cv2.line(final_image, (int(pnt[0]), int(pnt[1])), (int(match_pnt[0]), int(match_pnt[1])), (0, 255, 0), 2)

    cv.imshow("img", final_image)

    cv.waitKey(0)


if __name__ == '__main__':

    image_path = "1.tif"
    with rasterio.open(image_path) as src:
        transform = src.transform
        src_crs = src.crs

    path = 'image5.jpg'
    img = cv.imread(path)
    gray_img = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, thresh = cv.threshold(gray_img, 150, 255, cv.THRESH_BINARY)
    kernel = cv.getStructuringElement(cv.MORPH_RECT, (7, 7))
    eroded = cv.erode(thresh, kernel, iterations=1)
    os.makedirs('temp', exist_ok=True)
    cv.imwrite(f'temp/eroded.jpg', eroded)

    road_mask = np.zeros_like(eroded)
    contours, _ = cv.findContours(eroded, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    contours = [cnt for cnt in contours if cv.contourArea(cnt) > 90000]
    cv.drawContours(road_mask, contours, -1, 255, thickness=cv.FILLED)

    total_center = np.concatenate([find_center(cnt, road_mask=road_mask) for cnt in contours], axis=0)
    rand_ids = np.random.choice(len(total_center), size=len(total_center) // 2, replace=False)
    total_center = total_center[rand_ids]

    R_check_region = 150
    filter_centers = find_intersections(total_center, road_mask, img, R_check_region)

    reduce_radius = 150
    final_centers_list = reduce_points(filter_centers.squeeze(1).tolist(), reduce_radius)

    pts = np.expand_dims(np.array(final_centers_list), 1).reshape(-1, 2)

    img_gps_exif = gps_exift(path)

    for cx, cy in pts:
        lon, lat = convert_gps_pixel_to_geo(img_gps_exif, cx, cy)
        # if src_crs != "EPSG:4326":
        #     lon, lat = convert_coordinates(lon, lat, src_crs)
        print(f"Pixel coordinates: ({cx}, {cy}) -> Geo-coordinates: ({lon}, {lat})")

        cv.circle(img, (cx, cy), 1, (0, 0, 255), -1)
        coord_text = f"({lon:.6f}, {lat:.6f})"
        cv.putText(img, coord_text, (cx + 10, cy - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    cv.imwrite(f'temp/final_result.jpg', img)

    res = extract_and_draw_final(image_path, path, pts)

    match_image(path, pts, image_path, res)

    cv.waitKey(0)
    cv.destroyAllWindows()
