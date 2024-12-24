import cv2
import numpy as np

# GLOBAL VAR
scale_for_matching = 5
red_point_sz = 10


#

def extract_road(image, stname):
    blur = 1

    if image.shape[2] == 4:  # Trường hợp ảnh có kênh alpha (PNG)

        blur = 9
        alpha_channel = image[:, :, 3]

        # Tạo mặt nạ cho các vùng không trong suốt (alpha > 0)
        mask = alpha_channel > 0

        # Chuyển ảnh từ RGBA sang BGR (bỏ kênh alpha)
        image_bgr = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)

        # Sử dụng mặt nạ để giữ lại các phần không trong suốt
        image_bgr[~mask] = 0  # Đặt các pixel có alpha = 0 (vùng trong suốt) thành đen
        cv2.imwrite('temp/' + stname + '_mask.jpg', mask * 255)
    else:  # Trường hợp ảnh không có kênh alpha (BGR hoặc RGB)
        image_bgr = image
        alpha_channel = None

    # Chuyển ảnh thành grayscale để xử lý
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    # Làm mịn ảnh để giảm nhiễu

    blurred = cv2.GaussianBlur(gray, (blur, blur), 0)

    _, thresh = cv2.threshold(blurred, 127, 255, cv2.THRESH_BINARY)

    # Mở rộng các biên để kết nối các vùng sáng
    kernel = np.ones((5, 5), np.uint8)  # Kích thước của kernel
    dilated = cv2.dilate(thresh, kernel, iterations=1)

    # Tìm các vùng liên kết trong ảnh (connected components)
    num_labels, labels = cv2.connectedComponents(dilated)

    # Tạo một ảnh trắng cùng kích thước với ảnh gốc
    white_image = np.ones_like(image_bgr) * 255  # Tạo ảnh trắng

    # Vẽ các vùng con đường (connected components) lên ảnh trắng
    for i in range(1, num_labels):  # Bắt đầu từ 1 để bỏ qua nền (label 0)
        mask = (labels == i).astype(np.uint8) * 255
        # Kiểm tra nếu vùng sáng có đủ độ sáng để tô màu (ngưỡng sáng)
        region_brightness = np.mean(image_bgr[mask == 255])  # Tính trung bình độ sáng của vùng
        if region_brightness > 50:  # Lọc vùng tối (ngưỡng này có thể điều chỉnh)
            white_image[mask == 255] = (0, 255, 0)  # Gán màu xanh cho các vùng sáng đủ sáng

    if alpha_channel is not None:
        final_image = cv2.bitwise_and(white_image, white_image, mask=alpha_channel)
        final_image[alpha_channel == 0] = (255, 255, 255)
    else:
        final_image = white_image  # Nếu không có alpha_channel, chỉ sử dụng white_image

    cv2.imwrite('temp/' + stname + '_result.jpg', final_image)

    return white_image


def match_roads(image, template, src_rect, pts, rect):
    new_width = int(image.shape[1] * scale_for_matching)
    new_height = int(image.shape[0] * scale_for_matching)

    scaled_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LANCZOS4)
    cv2.imwrite("temp/scale.jpg", scaled_image)

    x, y, w, h = [int(v * scale_for_matching) for v in src_rect]
    roi = scaled_image[y:y + h, x:x + w]

    print('lot', y + h, x + w, new_width, new_height)

    # Tăng cường: Lấy biên dạng
    roi_edges = extract_road(roi, 'crop')
    template_edges = extract_road(template, 'temp')

    best_match = None
    best_value = -np.inf
    best_scale = 1

    # So khớp mẫu

    # for scal in np.arange(0.9,1.1,0.1):
    scal = 1
    scaled_template_edges = cv2.resize(template_edges, None, fx=scal, fy=scal, interpolation=cv2.INTER_LINEAR)

    # So khớp mẫu
    result = cv2.matchTemplate(roi_edges, scaled_template_edges, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(result)

    # Cập nhật nếu có kết quả tốt hơn
    if max_val > best_value:
        best_value = max_val
        best_match = (max_loc, scaled_template_edges.shape[::-1])
        best_scale = scal

    if best_match:
        top_left = best_match[0]
        template_size = best_match[1]
        height, width = template_size[2], template_size[1]
        bottom_right = (top_left[0] + width, top_left[1] + height)

        x = top_left[0]
        y = top_left[1]

        pls = [[x + int(_x * best_scale), y + int(_y * best_scale)] for _x, _y in rect]
        draw_bound(roi, pls)

        res = []

        for p in pts:
            _x = int(x + p[0] * best_scale)
            _y = int(y + p[1] * best_scale)

            res += [(_x, _y)]

            cv2.circle(roi, (_x, _y), red_point_sz, (0, 0, 255), -1)

    cv2.imwrite("temp/crop_scale.jpg", roi)

    # Lưu ảnh
    cv2.imwrite("temp/road_match.jpg", scaled_image)
    print(f"Ảnh đã lưu tại road_match.jpg với tỷ lệ tốt nhất: {best_scale} và điểm so khớp: {best_value}")
    return res
