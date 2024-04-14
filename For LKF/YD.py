# YD.py

## 사용시 필수 변경해줄것 ##
## 카메라 및 영상 선택 ##
## 프레임 크기 ##
## ROI범위 ##

#ROI입력시 유의사항
#프레임의 너비, 높이 확인
#프레임(W,H)1280,720 & ROI(x,y,w,h) 기준 예시
#프레임의 제일 하단 중앙에 w=200, h=200인 ROI -> x=540, y = 720-h/2 = 620, w = 200, h = 200
#ROI중심 위치: x_c = x좌표 + w/2 , y_c = y좌표 - h/2

import cv2
import numpy as np

# 노란색에 대한 고정된 HSV 범위 <- 색 변경 필요시 변경
def fixed_yellow_values():
    l_h = 20  # Lower Hue
    l_s = 100  # Lower Saturation
    l_v = 100  # Lower Value
    u_h = 30  # Upper Hue
    u_s = 255  # Upper Saturation
    u_v = 255  # Upper Value
    return (l_h, l_s, l_v, u_h, u_s, u_v)

#카메라 및 프레임 설정
def setup_camera():
    cap = cv2.VideoCapture('testVideo2.mp4')    ## 어떤 영상 및 카메라 쓸건지 ## 
    cap.set(3, 1280)  ## 프레임 너비 설정 ##
    cap.set(4, 720)   ## 프레임 높이 설정 ##
    if not cap.isOpened():
        print("비디오를 열 수 없습니다.")
        exit()
    return cap

def process_frame(cap, roi_x, roi_y, roi_width, roi_height, hsv_ranges):
    ret, frame = cap.read()
    if not ret:
        return None, False  # 프레임을 읽지 못하면 None과 False 반환

    # 좌우대칭
    #frame = cv2.flip(frame, 1)
    roi = frame[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    lower_range = np.array([hsv_ranges[0], hsv_ranges[1], hsv_ranges[2]])
    upper_range = np.array([hsv_ranges[3], hsv_ranges[4], hsv_ranges[5]])
    mask = cv2.inRange(hsv, lower_range, upper_range)
    result = cv2.bitwise_and(roi, roi, mask=mask)
    result[mask == 0] = (0, 0, 0)  # 나머지 영역은 검은색으로 채우기

    # 보라색 컨투어 추가
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(result, contours, -1, (255, 0, 255), 3)  # 보라색으로 컨투어 그리기
    return result, True

def main():
    cap = setup_camera()
    hsv_ranges = fixed_yellow_values()
    roi_x, roi_y, roi_width, roi_height = 240, 620, 800, 200    ## roi설정 부분 ##

    while True:
        result, success = process_frame(cap, roi_x, roi_y, roi_width, roi_height, hsv_ranges)
        if not success:
            break

        cv2.imshow('Processed', result)
        key = cv2.waitKey(1)
        if key == 27:  # ESC 키가 눌리면
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()