import cv2
import numpy as np
import math
from YD import setup_camera, process_frame, fixed_yellow_values

# Calculate the center of the contour
def find_contour_center(contour):
    moments = cv2.moments(contour)
    if moments["m00"] != 0:
        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
    else:
        cx, cy = 0, 0
    return cx, cy

# Calculate vector from bottom center to top center of the contour
def calculate_vector(contour, frame_height):
    cx, cy = find_contour_center(contour)
    start_point = (cx, frame_height)  # Bottom center
    end_point = (cx, cy)  # Top center of the contour
    vector = (end_point[0] - start_point[0], end_point[1] - start_point[1])
    return vector, start_point, end_point

# Convert to polar coordinates
def to_polar(vector):
    x, y = vector
    radius = math.hypot(x, y)
    angle = math.degrees(math.atan2(y, x))  # Convert atan result to degrees
    return radius, angle

# Calculate the control command based on angle
def calculate_command(angle):
    if 0 <= angle <= 75:
        return "Turn Right"
    elif 75 < angle < 105:
        return "Go Straight"
    else:
        return "Turn Left"

def main():
    cap = setup_camera()
    hsv_ranges = fixed_yellow_values()
    roi_x, roi_y, roi_width, roi_height = 240, 620, 800, 200

    while True:
        result, success = process_frame(cap, roi_x, roi_y, roi_width, roi_height, hsv_ranges)
        if not success:
            break

        # Contour processing and vector calculation
        mask = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)  # Convert result to grayscale
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            vector, start, end = calculate_vector(largest_contour, cap.get(4))  # Frame height from cap
            radius, angle = to_polar(vector)
            command = calculate_command(angle)
            print(f"Vector: {vector}, Polar Coordinates: (Radius: {radius}, Angle: {angle}), Command: {command}")

        cv2.imshow('Processed', result)
        key = cv2.waitKey(100)
        if key == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
