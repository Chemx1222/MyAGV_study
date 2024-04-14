#2. 찾아낸 HSV, ROI값에 해당하는 마스킹에 보라색 Contour생성
#프레임 크기(너비, 높이) 조정 필수
#좌우대칭 확인

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
cap = cv2.VideoCapture('testVideo2.mp4')

#프레임 크기 설정
cap.set(3, 1280)  # 프레임 너비 설정
cap.set(4, 720)   # 프레임 높이 설정

if not cap.isOpened():
    print("비디오를 열 수 없습니다.")
    exit()

# 사용자로부터 HSV 값과 ROI 값을 입력받음
(h_l_h, h_l_s, h_l_v, h_u_h, h_u_s, h_u_v), (roi_x, roi_y, roi_width, roi_height) = ask_for_values()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    #좌우대칭
    frame = cv2.flip(frame, 1)  # 프레임을 수평으로 뒤집기
    # 원본 이미지 표시
    cv2.imshow('Original', frame)

    # ROI 및 HSV 처리
    roi = frame[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_range = np.array([h_l_h, h_l_s, h_l_v])
    upper_range = np.array([h_u_h, h_u_s, h_u_v])
    mask = cv2.inRange(hsv, lower_range, upper_range)

    # 마스크를 이용해 흰색으로 표현할 영역 결정
    result = cv2.bitwise_and(roi, roi, mask=mask)
    result[mask == 0] = (0, 0, 0)  # 나머지 영역은 검은색으로 채우기

    # 보라색 컨투어 추가
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(result, contours, -1, (255, 0, 255), 3)  # 보라색으로 컨투어 그리기

    # 전체 이미지에 ROI 결과 복사 및 빨간색 사각형으로 ROI 시각화
    frame_with_roi = frame.copy()
    frame_with_roi[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width] = result
    cv2.rectangle(frame_with_roi, (roi_x, roi_y), (roi_x+roi_width, roi_y+roi_height), (0, 0, 255), 2)

    # 처리된 이미지 표시
    cv2.imshow('Processed', frame_with_roi)
    cv2.imshow('ROI Only', result)

    key = cv2.waitKey(1)
    if key == 27:  # ESC 키가 눌리면
        break

cap.release()
cv2.destroyAllWindows()
