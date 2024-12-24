import math

import cv2
import numpy as np


def keep_large_component_and_convert_to_rgba(binary_image, thres):
    """
    Retains components in the binary image with an area greater than the given threshold
    and converts the result to an RGBA image with a transparent background.

    Args:
        binary_image (numpy.ndarray): Binary input image.
        thres (int): Threshold area for retaining components.

    Returns:
        numpy.ndarray: RGBA image with large components retained.
    """
    # Find all connected components
    _, binary_thresh = cv2.threshold(cv2.cvtColor(binary_image, cv2.COLOR_RGBA2GRAY), 0, 255,
                                     cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(binary_thresh, connectivity=8)

    # Create an empty mask for components with area > thres
    large_components = np.zeros_like(binary_thresh, dtype=np.uint8)

    # Iterate through all components (excluding the background)
    for label in range(1, num_labels):
        if stats[label, cv2.CC_STAT_AREA] > thres:
            large_components[labels == label] = 255

    # Convert the binary image to an RGBA image
    rgba_image = cv2.cvtColor(large_components, cv2.COLOR_GRAY2RGBA)

    # Set alpha channel: make background transparent (where mask is 0)
    rgba_image[:, :, 3] = large_components

    return rgba_image


class DetectMatchingMap:
    scale_for_thresh = 5
    rect_in_map = None

    def __init__(self, ggmap_path, template_path, rect):
        self.rect_in_map = rect
        x, y, w, h = [v * self.scale_for_thresh for v in self.rect_in_map]

        self.ggmap_path = ggmap_path
        self.template_path = template_path

        self.ggmap_image = cv2.imread(self.ggmap_path, cv2.IMREAD_UNCHANGED)
        self.ggmap_image = self.ggmap_image[y:y + h, x:x + w]
        self.template_image = cv2.imread(self.template_path, cv2.IMREAD_UNCHANGED)

        self.template_image = self.thresholding_image(self.template_image, 0)

        # Giữ lại thành phần liên thông lớn (xóa bớt outline và giữ lại đường)
        self.template_image = keep_large_component_and_convert_to_rgba(self.template_image, 200)
        self.simply_road(self.template_image)

        self.ggmap_image = self.thresholding_image(self.ggmap_image, 1)

        self.overlay_with_best_match()

    def detect_roads(self, image, color_range, canny_thresholds, hough_params):
        original = image.copy()

        # Chuyển sang không gian màu HSV để lọc màu
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_color, upper_color = color_range
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Áp dụng mask để giữ lại vùng có màu đặc trưng của đường
        color_filtered = cv2.bitwise_and(image, image, mask=mask)

        # Chuyển ảnh đã lọc màu sang xám
        gray = cv2.cvtColor(color_filtered, cv2.COLOR_BGR2GRAY)

        # Làm mờ để giảm nhiễu
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Phát hiện cạnh bằng Canny
        canny_threshold1, canny_threshold2 = canny_thresholds
        edges = cv2.Canny(blurred, canny_threshold1, canny_threshold2)

        # Phát hiện đường thẳng bằng Hough Transform
        rho, theta, threshold, min_line_length, max_line_gap = hough_params
        lines = cv2.HoughLinesP(edges, rho, theta, threshold, minLineLength=min_line_length, maxLineGap=max_line_gap)

        # Vẽ các đường phát hiện được lên ảnh gốc
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Kết hợp các kết quả
        combined = cv2.addWeighted(original, 0.5, image, 0.5, 0)

        return edges, color_filtered, combined

    def thresholding_image(self, image, index=0):
        # Dải màu đặc trưng của đường (trong không gian HSV)
        color_range = (np.array([0, 0, 200]), np.array([180, 50, 255]))  # Ví dụ cho màu trắng

        # Ngưỡng Canny
        canny_thresholds = (50, 150)

        # Tham số Hough Transform
        hough_params = (1, np.pi / 180, 100, 50, 10)

        # Phát hiện đường
        edges, color_filtered, combined_result = self.detect_roads(image, color_range, canny_thresholds, hough_params)

        # cv2.imwrite(f"edges_result_{index}.jpg", edges)
        cv2.imwrite(f"color_filtered_{index}.jpg", color_filtered)

        return color_filtered

    def connect_nearby_white_areas(self, image, color_range, area_threshold, connect_threshold, blur_kernel_size):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Lọc màu trắng
        lower_color, upper_color = color_range
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Làm nhòe ảnh bằng Gaussian Blur
        blurred_mask = cv2.GaussianBlur(mask, (blur_kernel_size, blur_kernel_size), 0)

        # Áp dụng ngưỡng để giữ lại vùng trắng lớn
        _, binary_mask = cv2.threshold(blurred_mask, 127, 255, cv2.THRESH_BINARY)

        # Tính khoảng cách giữa các điểm trong ảnh
        dist_transform = cv2.distanceTransform(binary_mask, cv2.DIST_L2, 5)

        # Kết nối các vùng gần nhau bằng dilation (nếu khoảng cách giữa chúng nhỏ hơn ngưỡng)
        dilated_mask = cv2.dilate(binary_mask, None, iterations=1)

        # Tìm các vùng liên thông
        contours, _ = cv2.findContours(dilated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Tạo một mask rỗng để giữ lại các vùng đạt ngưỡng
        filtered_mask = np.zeros_like(dilated_mask)

        for contour in contours:
            # Tính diện tích vùng
            area = cv2.contourArea(contour)
            # if area > area_threshold:
            # Giữ lại vùng nếu diện tích lớn hơn ngưỡng
            cv2.drawContours(filtered_mask, [contour], -1, 255, thickness=cv2.FILLED)

        # Áp dụng mask để giữ lại vùng đạt yêu cầu trên ảnh gốc
        filtered_image = cv2.bitwise_and(image, image, mask=filtered_mask)

        return filtered_image, filtered_mask

    def simply_road(self, image):
        # Đường dẫn tới hình ảnh
        # image_path = "color_filtered.jpg"

        # Dải màu đặc trưng của đường (trong không gian HSV)
        color_range = (np.array([0, 0, 200]), np.array([180, 50, 255]))  # Lọc màu trắng

        # Ngưỡng diện tích (ví dụ: 50 pixel vuông)
        area_threshold = 1

        # Ngưỡng kết nối (khoảng cách tối đa giữa các vùng trắng để kết nối)
        connect_threshold = 50  # Pixels

        # Kích thước kernel cho làm nhòe
        blur_kernel_size = 15

        # Lọc và kết nối các vùng trắng gần nhau
        filtered_image, filtered_mask = self.connect_nearby_white_areas(
            image, color_range, area_threshold, connect_threshold, blur_kernel_size
        )

        cv2.imwrite("temp/simply_road.jpg", filtered_mask)

        return filtered_mask

    def rotated(self, image, angle):
        (h, w) = image.shape[:2]

        # Define the center of rotation
        center = (w // 2, h // 2)

        # Define the rotation matrix (angle in degrees)
        angle = math.degrees(angle)
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1)

        # Perform the rotation
        rotated_image = cv2.warpAffine(image, rotation_matrix, (w, h))

        return rotated_image

    # Thay vì chỉ scale tỉ lệ, xoay ảnh để tìm ảnh match nhất với ảnh gốc
    def match_with_scale_and_rotate(self):
        self.best_match_score = -1  # Điểm tốt nhất
        self.best_scale = 1.0  # Tỷ lệ tốt nhất
        self.best_angle = 0  # Tỷ lệ tốt nhất
        self.best_top_left = None  # Vị trí tốt nhất
        self.best_template = self.template_image

        for scale in np.arange(0.8, 1.2, 0.05):
            # Thực hiện resize template theo scale
            for angle in np.arange(-0.2, 0.2, 0.05):
                scaled_template = cv2.resize(self.template_image, None, fx=scale, fy=scale,
                                             interpolation=cv2.INTER_LANCZOS4)

                rotated_scale_template = self.rotated(scaled_template, angle)

                template_gray = cv2.cvtColor(rotated_scale_template, cv2.COLOR_BGR2GRAY)

                roi_gray = cv2.cvtColor(self.ggmap_image, cv2.COLOR_BGR2GRAY)

                # So khớp mẫu
                result = cv2.matchTemplate(roi_gray, template_gray, cv2.TM_CCOEFF_NORMED)

                # Tìm vị trí và điểm tốt nhất
                _, max_val, _, max_loc = cv2.minMaxLoc(result)

                if max_val > self.best_match_score:  # Nếu điểm tốt hơn, cập nhật lại
                    self.best_match_score = max_val
                    self.best_scale = scale
                    self.best_angle = angle
                    self.best_top_left = max_loc
                    self.best_template = scaled_template

        return self.best_top_left, self.best_template, self.best_scale, self.best_match_score, self.best_angle

    def match_with_scaling(self):
        self.best_match_score = -1  # Điểm tốt nhất
        self.best_scale = 1.0  # Tỷ lệ tốt nhất
        self.best_top_left = None  # Vị trí tốt nhất
        self.best_template = self.template_image

        for scale in np.arange(0.8, 1.2, 0.05):
            # Thực hiện resize template theo scale
            scaled_template = cv2.resize(self.template_image, None, fx=scale, fy=scale,
                                         interpolation=cv2.INTER_LANCZOS4)
            template_gray = cv2.cvtColor(scaled_template, cv2.COLOR_BGR2GRAY)

            cv2.imwrite("template_gray.jpg", template_gray)

            roi_gray = cv2.cvtColor(self.ggmap_image, cv2.COLOR_BGR2GRAY)

            cv2.imwrite("roi_gray.jpg", roi_gray)

            # So khớp mẫu
            result = cv2.matchTemplate(roi_gray, template_gray, cv2.TM_CCOEFF_NORMED)

            # Tìm vị trí và điểm tốt nhất
            _, max_val, _, max_loc = cv2.minMaxLoc(result)

            if max_val > self.best_match_score:  # Nếu điểm tốt hơn, cập nhật lại
                self.best_match_score = max_val
                self.best_scale = scale
                self.best_top_left = max_loc
                self.best_template = scaled_template

        return self.best_top_left, self.best_template, self.best_scale, self.best_match_score

    def overlay_with_best_match(self):
        cv2.imwrite('temp/img.jpg', self.ggmap_image)

        # Tìm tỷ lệ khớp tốt nhất
        best_top_left, best_template, best_scale, best_match_score, best_angle = self.match_with_scale_and_rotate()
        # best_top_left, best_template, best_scale, best_match_score = self.match_with_scaling()

        if best_top_left is not None:
            print(f"Best Match Score: {best_match_score}, Best Scale: {best_scale}")

            # Vẽ hình chữ nhật lên ảnh ggmap_image
            top_left = best_top_left

            self.result = (top_left[0] / self.scale_for_thresh, top_left[1] / self.scale_for_thresh)

            print(self.result)

            bottom_right = (top_left[0] + best_template.shape[1], top_left[1] + best_template.shape[0])

            cv2.rectangle(self.ggmap_image, top_left, bottom_right, (0, 255, 0), 2)

            # Overlay template lên ggmap_image với độ mờ 50%
            overlay_image = self.ggmap_image.copy()
            for c in range(0, 3):  # Duyệt qua 3 kênh màu (BGR)
                a = 0.5 * best_template[0:best_template.shape[0], 0:best_template.shape[1], c]
                b = 0.5 * overlay_image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], c]

                overlay_image[top_left[1]:bottom_right[1], top_left[0]:bottom_right[0], c] = a + b

            cv2.imwrite('temp/best_result.jpg', overlay_image)

            return top_left
        else:
            print("Không tìm thấy khớp mẫu")

    def transform_points(self, p):
        return (self.result[0] + p[0], self.result[1] + p[1])


if __name__ == '__main__':
    cls = DetectMatchingMap("temp/scale.jpg", "temp/img_2.png", [260, 65, 400, 350])
