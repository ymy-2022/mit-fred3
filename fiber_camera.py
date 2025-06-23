import time
import sys
import cv2
import numpy as np
from typing import Tuple
from PyQt5.QtWidgets import QWidget, QLabel, QDoubleSpinBox
from PyQt5.QtGui import QImage, QPixmap

from database import Database

class FiberCamera(QWidget):
    """Process video from camera to obtain the fiber diameter and display it"""
    def __init__(self, target_diameter: QDoubleSpinBox, gui) -> None:
        super().__init__()
        self.raw_image = QLabel()
        self.canny_image = QLabel()
        self.processed_image = QLabel()
        self.target_diameter = target_diameter
        self.capture = cv2.VideoCapture(0)
        self.gui = gui
        self.diameter_coefficient = Database.get_calibration_data("diameter_coefficient")
        self.previous_time = 0.0

        # 实时参数（与 UI 联动）
        self.erode_enabled = True
        self.dilate_enabled = True
        self.gaussian_enabled = True
        self.binary_enabled = True
        self.canny_lower = 100
        self.canny_upper = 250
        self.hough_threshold = 30

    def camera_loop(self) -> None:
        """Loop to capture and process frames from the camera"""
        current_time = time.time()
        success, frame = self.capture.read()
        if not success:
            return

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, _, _ = frame.shape
        frame = frame[height//4:3*height//4, :]  # 保留中间部分
        edges, binary_frame = self.get_edges(frame)
        detected_lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180, self.hough_threshold,
            minLineLength=30, maxLineGap=100
        )
        fiber_diameter = self.get_fiber_diameter(detected_lines)
        frame_with_lines = self.plot_lines(frame.copy(), detected_lines)

        Database.camera_timestamps.append(current_time)
        Database.diameter_readings.append(fiber_diameter)
        Database.diameter_setpoint.append(self.target_diameter.value())
        Database.diameter_delta_time.append(current_time - self.previous_time)
        self.previous_time = current_time

        # 显示原图
        image_for_gui = QImage(frame_with_lines, frame_with_lines.shape[1], frame_with_lines.shape[0],
                               QImage.Format_RGB888)
        self.raw_image.setPixmap(QPixmap(image_for_gui))

        # 显示Canny（虽然没在UI显示，但保留代码）
        image_for_gui = QImage(edges, edges.shape[1], edges.shape[0], QImage.Format_Grayscale8)
        self.canny_image.setPixmap(QPixmap(image_for_gui))

        # 显示二值图像（处理图）
        image_for_gui = QImage(binary_frame, binary_frame.shape[1], binary_frame.shape[0], QImage.Format_Grayscale8)
        self.processed_image.setPixmap(QPixmap(image_for_gui))

    def get_edges(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """根据UI参数处理图像，返回边缘图和二值图"""
        gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        kernel = np.ones((5, 5), np.uint8)

        if self.erode_enabled:
            gray = cv2.erode(gray, kernel, iterations=2)
        if self.dilate_enabled:
            gray = cv2.dilate(gray, kernel, iterations=2)
        if self.gaussian_enabled:
            gray = cv2.GaussianBlur(gray, (5, 5), 0)
        if self.binary_enabled:
            _, binary_frame = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
        else:
            binary_frame = gray.copy()

        # 用二值图还是灰度图做Canny
        canny_input = binary_frame if self.binary_enabled else gray
        edges = cv2.Canny(
            canny_input,
            self.canny_lower, self.canny_upper, apertureSize=3
        )
        return edges, binary_frame

    def get_fiber_diameter(self, lines):
        """Get the fiber diameter from the edges detected in the image"""
        leftmost_min = sys.maxsize
        leftmost_max = 0
        rightmost_min = sys.maxsize
        rightmost_max = 0
        if lines is None or len(lines) <= 1:
            return 0
        for line in lines:
            x0, _, x1, _ = line[0]
            leftmost_min = min(leftmost_min, x0, x1)
            leftmost_max = max(leftmost_max, min(x0, x1))
            rightmost_min = min(rightmost_min, max(x0, x1))
            rightmost_max = max(rightmost_max, x0, x1)
        return (((leftmost_max - leftmost_min) + (rightmost_max - rightmost_min))
                / 2 * self.diameter_coefficient)

    def get_fiber_diameter_in_pixels(self, lines):
        leftmost_min = sys.maxsize
        leftmost_max = 0
        rightmost_min = sys.maxsize
        rightmost_max = 0
        if lines is None or len(lines) <= 1:
            return 0
        for line in lines:
            x0, _, x1, _ = line[0]
            leftmost_min = min(leftmost_min, x0, x1)
            leftmost_max = max(leftmost_max, min(x0, x1))
            rightmost_min = min(rightmost_min, max(x0, x1))
            rightmost_max = max(rightmost_max, x0, x1)
        return (((leftmost_max - leftmost_min) + (rightmost_max - rightmost_min)) / 2)

    def plot_lines(self, frame, lines):
        """Plot the detected lines on the frame"""
        if lines is not None:
            for line in lines:
                x0, y0, x1, y1 = line[0]
                cv2.line(frame, (x0, y0), (x1, y1), (255, 0, 0), 2)
        return frame

    def calibrate(self):
        """Calibrate the camera"""
        num_samples = 50
        accumulated_diameter = 0
        valid_samples = 0

        for _ in range(num_samples):
            success, frame = self.capture.read()
            if not success:
                continue
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            edges, _ = self.get_edges(frame)
            detected_lines = cv2.HoughLinesP(edges, 1, np.pi / 180, self.hough_threshold,
                                             minLineLength=30, maxLineGap=100)
            fiber_diameter = self.get_fiber_diameter_in_pixels(detected_lines)
            if fiber_diameter is not None:
                accumulated_diameter += fiber_diameter
                valid_samples += 1

        if valid_samples > 0:
            average_diameter = accumulated_diameter / valid_samples
        else:
            average_diameter = 1  # 防止除零

        print(f"Average width of wire: {average_diameter} pixels")
        self.diameter_coefficient = 0.94 / average_diameter
        print(f"Diameter_coeff: {self.diameter_coefficient} ")
        Database.update_calibration_data("diameter_coefficient", str(self.diameter_coefficient))

    def closeEvent(self, event):
        """Close the camera when the window is closed"""
        self.capture.release()
        event.accept()
