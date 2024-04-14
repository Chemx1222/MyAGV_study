#1. HSV값, ROI찾기
#노랑색 H:20~40, S:100~255, V:100~255
#프레임 크기(너비, 높이) 조정 필수
#좌우대칭 확인

import cv2
import numpy as np

def on_trackbar_change(dummy=None):
    pass

# 웹캠에서 비디오 캡처 시작
cap = cv2.VideoCapture(0)

#프레임 크기 설정
cap.set(3, 1280)  # 프레임 너비 설정
cap.set(4, 720)   # 프레임 높이 설정

if not cap.isOpened():
    print("비디오를 열 수 없습니다.")
    exit()

# 트랙바를 위한 윈도우 생성
cv2.namedWindow('Trackbars')
cv2.resizeWindow('Trackbars', 600, 400)

# HSV 트랙바 생성
cv2.createTrackbar('Lower Hue', 'Trackbars', 0, 179, on_trackbar_change)
cv2.createTrackbar('Lower Saturation', 'Trackbars', 0, 255, on_trackbar_change)
cv2.createTrackbar('Lower Value', 'Trackbars', 0, 255, on_trackbar_change)
cv2.createTrackbar('Upper Hue', 'Trackbars', 179, 179, on_trackbar_change)
cv2.createTrackbar('Upper Saturation', 'Trackbars', 255, 255, on_trackbar_change)
cv2.createTrackbar('Upper Value', 'Trackbars', 255, 255, on_trackbar_change)

# ROI 트랙바 생성
cv2.createTrackbar('ROI X', 'Trackbars', 100, 1280, on_trackbar_change)
cv2.createTrackbar('ROI Y', 'Trackbars', 100, 720, on_trackbar_change)
cv2.createTrackbar('ROI Width', 'Trackbars', 100, 1280, on_trackbar_change)
cv2.createTrackbar('ROI Height', 'Trackbars', 100, 720, on_trackbar_change)

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    #좌우대칭
    frame = cv2.flip(frame, 1)  # 프레임을 수평으로 뒤집기

    # 트랙바에서 값 읽기
    l_h = cv2.getTrackbarPos('Lower Hue', 'Trackbars')
    l_s = cv2.getTrackbarPos('Lower Saturation', 'Trackbars')
    l_v = cv2.getTrackbarPos('Lower Value', 'Trackbars')
    u_h = cv2.getTrackbarPos('Upper Hue', 'Trackbars')
    u_s = cv2.getTrackbarPos('Upper Saturation', 'Trackbars')
    u_v = cv2.getTrackbarPos('Upper Value', 'Trackbars')
    roi_x = cv2.getTrackbarPos('ROI X', 'Trackbars')
    roi_y = cv2.getTrackbarPos('ROI Y', 'Trackbars')
    roi_width = cv2.getTrackbarPos('ROI Width', 'Trackbars')
    roi_height = cv2.getTrackbarPos('ROI Height', 'Trackbars')

    # ROI 및 HSV 처리
    roi = frame[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_range = np.array([l_h, l_s, l_v])
    upper_range = np.array([u_h, u_s, u_v])
    mask = cv2.inRange(hsv, lower_range, upper_range)
    result = cv2.bitwise_and(roi, roi, mask=mask)
    result[mask == 0] = (0, 0, 0)

    # 전체 이미지에 ROI 결과 복사 및 빨간색 사각형으로 ROI 시각화
    frame_with_roi = frame.copy()
    frame_with_roi[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width] = result
    cv2.rectangle(frame_with_roi, (roi_x, roi_y), (roi_x+roi_width, roi_y+roi_height), (0, 0, 255), 2)

    cv2.imshow('Processed', frame_with_roi)
    cv2.imshow('ROI Only', result)

    key = cv2.waitKey(1)
    if key == 27:  # ESC 키가 눌리면
        break

    if key == ord('s'):  # 's'를 누르면 설정 저장
        hsv_values = [l_h, l_s, l_v, u_h, u_s, u_v]
        roi_values = [roi_x, roi_y, roi_width, roi_height]
        print(f'HSV_H: {l_h} ~ {u_h}')
        print(f'HSV_S: {l_s} ~ {u_s}')
        print(f'HSV_V: {l_v} ~ {u_v}')
        print(f'ROI_x, y: {roi_x, roi_y}')
        print(f'ROI_W, H: {roi_width, roi_height}')
        np.save('HSV_value.npy', hsv_values)
        np.save('ROI_value.npy', roi_values)
        break

cap.release()
cv2.destroyAllWindows()
