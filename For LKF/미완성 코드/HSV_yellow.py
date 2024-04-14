#1. HSV값 찾기

import cv2
import numpy as np
import time

def nothing(x):
    pass  # 콜백 함수 정의, 트랙바 이벤트에 사용

# 웹캠에서 비디오 캡처 시작
cap = cv2.VideoCapture(0)
cap.set(3, 1280)  # 프레임 너비 설정
cap.set(4, 720)   # 프레임 높이 설정

# 트랙바용 창 생성
cv2.namedWindow("Trackbars")

# 노란색 추출을 위한 트랙바 초기 설정
cv2.createTrackbar("L - H", "Trackbars", 20, 179, nothing)  # Hue 하한 설정
cv2.createTrackbar("L - S", "Trackbars", 100, 255, nothing)  # Saturation 하한 설정
cv2.createTrackbar("L - V", "Trackbars", 100, 255, nothing)  # Value 하한 설정
cv2.createTrackbar("U - H", "Trackbars", 40, 179, nothing)  # Hue 상한 설정
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)  # Saturation 상한 설정
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)  # Value 상한 설정

while True:
    ret, frame = cap.read()  # 프레임 읽기
    if not ret:
        break
    frame = cv2.flip(frame, 1)  # 프레임 수평 뒤집기

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 프레임을 HSV 색공간으로 변환
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")

    lower_range = np.array([l_h, l_s, l_v])  # 하한 범위 설정
    upper_range = np.array([u_h, u_s, u_v])  # 상한 범위 설정

    mask = cv2.inRange(hsv, lower_range, upper_range)  # 색상 범위에 따른 마스크 생성
    res = cv2.bitwise_and(frame, frame, mask=mask)  # 마스크를 적용한 결과 이미지 생성

    mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)  # 마스크를 3채널 이미지로 변환
    stacked = np.hstack((mask_3, frame, res))  # 마스크, 원본, 결과 이미지를 수평으로 합성

    cv2.imshow('Trackbars', cv2.resize(stacked, None, fx=0.4, fy=0.4))  # 결과 이미지 표시

    key = cv2.waitKey(1)  # 1밀리초 동안 키 입력 대기
    if key == 27:  # ESC 키가 눌리면 종료
        break
    if key == ord('s'):  # 's' 키가 눌리면
        thearray = [[l_h, l_s, l_v], [u_h, u_s, u_v]]  # 현재 트랙바의 HSV 범위 값을 배열로 저장
        print(f'HSV_Low: {thearray[0]}')  # HSV 하한 값 출력
        print(f'HSV_Upper: {thearray[1]}')  # HSV 상한 값 출력
        np.save('hsv_value', thearray)  # 배열을 파일로 저장
        break

cap.release, cv2.destroyAllWindows()  # 모든 창 닫기

#이 코드는 웹캠을 통해 실시간으로 노란색 선을 검출하도록 조정되었습니다. 
#HSV 색상 범위는 노란색의 특성에 맞추어 조절되었으며, 사용자가 트랙바를 조작하여 색상 범위를 직접 조절할 수 있습니다. 
#노란색 객체가 검출되면 화면에 결과가 표시됩니다. 필요에 따라 
#'s' 키를 누르면 현재 설정된 HSV 범위를 저장할 수 있습니다.
