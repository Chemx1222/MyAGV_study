#찾아낸 HSV, ROI를 적용시켜서 해당 부분 마스킹


import cv2
import numpy as np

def ask_for_values():
    print("HSV 값 입력: (Hue, Saturation, Value)")
    l_h = int(input("Lower Hue: "))
    l_s = int(input("Lower Saturation: "))
    l_v = int(input("Lower Value: "))
    u_h = int(input("Upper Hue: "))
    u_s = int(input("Upper Saturation: "))
    u_v = int(input("Upper Value: "))

    print("\nROI 값 입력: (x, y, width, height)")
    x = int(input("X: "))
    y = int(input("Y: "))
    width = int(input("Width: "))
    height = int(input("Height: "))
    
    return (l_h, l_s, l_v, u_h, u_s, u_v), (x, y, width, height)

# 웹캠에서 비디오 캡처 시작
cap = cv2.VideoCapture(0)
cap.set(3, 1280)  # 프레임 너비 설정
cap.set(4, 720)   # 프레임 높이 설정

if not cap.isOpened():
    print("카메라를 열 수 없습니다.")
    exit()

# 사용자로부터 HSV 값과 ROI 값을 입력받음
(h_l_h, h_l_s, h_l_v, h_u_h, h_u_s, h_u_v), (roi_x, roi_y, roi_width, roi_height) = ask_for_values()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    #좌우반전
    frame = cv2.flip(frame, 1)  # 프레임 수평 뒤집기

    # 원본 이미지 표시
    cv2.imshow('Original', frame)

    # ROI 및 HSV 처리
    roi = frame[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_range = np.array([h_l_h, h_l_s, h_l_v])
    upper_range = np.array([h_u_h, h_u_s, h_u_v])
    mask = cv2.inRange(hsv, lower_range, upper_range)
    mask_inv = cv2.bitwise_not(mask)

    # ROI 내에서 흰색과 회색 처리
    roi_result = cv2.bitwise_and(roi, roi, mask=mask)
    roi_gray = cv2.cvtColor(cv2.bitwise_and(roi, roi, mask=mask_inv), cv2.COLOR_BGR2GRAY)
    roi_gray_3ch = cv2.cvtColor(roi_gray, cv2.COLOR_GRAY2BGR)
    combined = cv2.add(roi_result, roi_gray_3ch)

    # Contour 처리
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(combined, contours, -1, (255, 0, 255), 3)

    # 전체 이미지에 ROI 결과 복사 및 빨간색 사각형으로 ROI 시각화
    frame_with_roi = frame.copy()
    frame_with_roi[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width] = combined
    cv2.rectangle(frame_with_roi, (roi_x, roi_y), (roi_x+roi_width, roi_y+roi_height), (0, 0, 255), 2)

    # 처리된 이미지 표시
    cv2.imshow('Processed', frame_with_roi)
    cv2.imshow('ROI Only', combined)

    key = cv2.waitKey(1)
    if key == 27:  # ESC 키가 눌리면
        break

cap.release()
cv2.destroyAllWindows()
