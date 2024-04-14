#실행전 해줄것
#1. AGV 설정: agv.connect()
#2. 비디오 소스 설정: setup_camera()
#3. Threshold, speed, Rotation_speed 값 설정
#4. ROI 설정: ROI(관심 영역) 설정(roi_x, roi_y, roi_width, roi_height)은 검출하려는 라인에 맞게 조정
#5. witkey(숫자) 조절


from YD import process_frame, setup_camera, fixed_yellow_values
from pymycobot.myagv import MyAgv
import cv2

# Initialize the AGV
agv = MyAgv()   # AGV제어를 위한 AGV객체 생성
agv.connect()   # AGV 연결

def count_pixels(result):
    #함수는 처리된 이미지에서 특정 조건을 만족하는 픽셀 수를 세는 기능을 수행합니다. 
    #result 매개변수는 이미지 데이터를 담고 있습니다.
    # Count the number of non-black pixels on the left and right sides of the center
    height, width, _ = result.shape #result 이미지의 높이(height), 너비(width), 채널 수를 추출합니다. 
                                    #여기서 _는 채널 수를 무시합니다(일반적으로 RGB 이미지는 3채널).
    center_x = width // 2           #이미지의 중앙 x 좌표를 계산
    left_count = cv2.countNonZero(cv2.cvtColor(result[:, :center_x], cv2.COLOR_BGR2GRAY))
    right_count = cv2.countNonZero(cv2.cvtColor(result[:, center_x:], cv2.COLOR_BGR2GRAY))
    #이미지의 왼쪽 및 오른쪽 부분을 회색조로 변환한 후, 0이 아닌 픽셀(검은색이 아닌 픽셀)의 수를 각각 세어 left_count와 right_count에 저장

    return left_count, right_count  #계산된 왼쪽 및 오른쪽 픽셀 수를 반환

def drive_agv(left_pixels, right_pixels):       #계산된 픽셀 수에 기반하여 AGV의 주행 방향을 결정하고 명령
    
    #주행 결정에 사용될 파라미터들을 설정
    threshold = 50  ## 주행 방향 결정의 민감도
    speed = 10 ## 이동 속도 설정
    rotation_speed = 5  ## 회전 속도 설정

    if abs(left_pixels - right_pixels) <= threshold: #좌우의 픽셀수 차이가 threshold수 보다 작으면 직진
        direction = "Straight"
        agv.go_ahead(speed)

    elif abs(left_pixels - right_pixels) > threshold:   #좌우의 픽셀수 차이가 threshold수 보다 큼

        if left_pixels > right_pixels:                  #좌측의 픽셀수가 우측의 픽셀수보다 많으면 좌회전
            direction = "Left Turn"
            agv.counterclockwise_rotation(rotation_speed)
        
        else:                                           #우측의 픽셀수가 좌측의 픽셀수보다 많으면 우회전
            direction = "Right Turn"
            agv.clockwise_rotation(rotation_speed)


    print(f"Driving {direction}: Left Pixels = {left_pixels}, Right Pixels = {right_pixels}")
    #현재 주행 방향, 왼쪽 픽셀수, 오른쪽 픽셀수 출력


def main():                     #AGV의 주행 로직을 실행
    cap = setup_camera()        #함수를 호출하여 비디오 캡처 객체 cap을 생성합니다. 이 객체는 카메라 또는 비디오 파일로부터 프레임을 캡처하는 데 사용
    hsv_ranges = fixed_yellow_values()      #함수를 호출하여 노란색을 검출하기 위한 HSV 색상 범위를 hsv_ranges 변수에 저장
    roi_x, roi_y, roi_width, roi_height = 240, 620, 800, 200    ## 관심 영역(Region of Interest, ROI)의 위치와 크기를 설정합니다. 
                                                                # 여기서 roi_x, roi_y는 ROI의 시작 좌표이며, roi_width, roi_height는 ROI의 너비와 높이

    while True: #무한 반복문
        result, success = process_frame(cap, roi_x, roi_y, roi_width, roi_height, hsv_ranges)
        # 현재 프레임을 처리합니다.
        #현재 캡처된 프레임에 대해 ROI 설정(x, y, w, h), HSV 변환, 노란색 마스킹 등의 작업을 수행
        # 결과 이미지 result와 성공 여부 success를 반환합니다.

        #프레임 처리에 실패시 반복 해제
        if not success: 
            break

        left_pixels, right_pixels = count_pixels(result)    #처리된 이미지에서 왼쪽과 오른쪽 영역에 있는 픽셀 수를 각각 left_pixels, right_pixels로 반환
        drive_agv(left_pixels, right_pixels)                #왼쪽과 오른쪽 픽셀 수에 기반하여 AGV의 주행 방향을 결정하고 관련 명령을 실행

        cv2.imshow('AGV Line Tracing View', result)         #result 이미지를 'AGV Line Tracing View'라는 창에 표시합니다. 이 창은 처리된 결과를 시각적으로 확인
                                                            # 시각화 불필요시 제거
        
        if cv2.waitKey(1) & 0xFF == ord('q'):               ##cv2.waitKey(1) 함수는 1밀리초 동안 키 입력을 대기하고
                                                            #사용자가 'q' 키를 누르면 루프에서 빠져나옵니다. 0xFF는 키 입력을 올바르게 처리하기 위한 마스크
            break

    cap.release()               #비디오 캡처 객체를 해제
    cv2.destroyAllWindows()     #모든 OpenCV 창을 닫
    agv.disconnect()            #AGV와의 연결을 종료

#스크립트가 직접 실행될 때만 main() 함수를 호출하도록 합니다. 
#이는 모듈로서 스크립트를 다른 스크립트에서 임포트할 때 실행을 방지하기 위해 사용
if __name__ == "__main__":      
    main()
