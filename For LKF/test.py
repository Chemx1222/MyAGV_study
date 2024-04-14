#ROI입력시 유의사항
#프레임의 너비, 높이 확인
#프레임(W,H)1280,720 & ROI(x,y,w,h) 기준 예시
#프레임의 제일 하단 중앙에 w=200, h=200인 ROI -> x=540, y = 720-h/2 = 620, w = 200, h = 200
#ROI중심 위치: x = x좌표 + w/2 , y = y좌표 - h/2

import cv2
import numpy as np

def fixed_yellow_values():
    # 노란색에 대한 고정된 HSV 범위
    l_h = 20  # Lower Hue
    l_s = 100  # Lower Saturation
    l_v = 100  # Lower Value
    u_h = 30  # Upper Hue
    u_s = 255  # Upper Saturation
    u_v = 255  # Upper Value
    
    print("노란색 추출을 위한 고정된 HSV 값 사용")
    print("Lower HSV: (", l_h, ",", l_s, ",", l_v, ")")
    print("Upper HSV: (", u_h, ",", u_s, ",", u_v, ")")
    
    return (l_h, l_s, l_v, u_h, u_s, u_v)

def ask_for_roi():
    print("\nROI 값 입력: (x, y, width, height)")
    x = int(input("X: "))
    y = int(input("Y: "))
    width = int(input("Width: "))
    height = int(input("Height: "))
    
    return (x, y, width, height)

# 웹캠에서 비디오 캡처 시작
cap = cv2.VideoCapture('testVideo2.mp4')

# 프레임 크기 설정
cap.set(3, 1280)  # 프레임 너비 설정
cap.set(4, 720)   # 프레임 높이 설정

if not cap.isOpened():
    print("비디오를 열 수 없습니다.")
    exit()

# 고정된 노란색 HSV 값 설정
(h_l_h, h_l_s, h_l_v, h_u_h, h_u_s, h_u_v) = fixed_yellow_values()

# 사용자로부터 ROI 값 입력 받음
(roi_x, roi_y, roi_width, roi_height) = ask_for_roi()

while True:
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break
    
    # 좌우대칭
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
