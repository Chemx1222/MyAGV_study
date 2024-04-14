#실행전 해줄것
#1. AGV 설정: agv.connect()
#2. 비디오 소스 설정: setup_camera()
#3. Threshold, speed, Rotation_speed 값 설정
#4. ROI 설정: ROI(관심 영역) 설정(roi_x, roi_y, roi_width, roi_height)은 검출하려는 라인에 맞게 조정
#5. waitkey(숫자) 조절

from YD import process_frame, setup_camera, fixed_yellow_values
from pymycobot.myagv import MyAgv
import cv2
import time

# AGV 초기화 및 연결
agv = MyAgv()
agv.connect()   ##1. AGV연결 설정

def count_pixels(result):
    """
    주어진 이미지에서 왼쪽과 오른쪽에 있는 비검은색 픽셀의 수를 계산
    :param result: 처리된 이미지
    :return: 왼쪽, 오른쪽 픽셀 수
    """
    height, width, _ = result.shape
    center_x = width // 2
    left_count = cv2.countNonZero(cv2.cvtColor(result[:, :center_x], cv2.COLOR_BGR2GRAY))
    right_count = cv2.countNonZero(cv2.cvtColor(result[:, center_x:], cv2.COLOR_BGR2GRAY))
    return left_count, right_count

def drive_agv(left_pixels, right_pixels, last_direction, speed=10, rotation_speed=5):
    """
    픽셀 수를 바탕으로 AGV의 주행 방향 결정 및 제어
    :param left_pixels: 왼쪽 픽셀 수
    :param right_pixels: 오른쪽 픽셀 수
    :param last_direction: 마지막 주행 방향
    :param speed: 주행 속도
    :param rotation_speed: 회전 속도
    :return: 새로운 주행 방향
    """
    threshold = 50      ##3. Threshold 설정
    if left_pixels == 0 and right_pixels == 0:
        return last_direction  # 노란색 선이 검출되지 않으면 이전 방향 유지
    if abs(left_pixels - right_pixels) <= threshold:
        direction = "Straight"
        agv.go_ahead(speed)
    elif left_pixels > right_pixels:
        direction = "Left Turn"
        agv.counterclockwise_rotation(rotation_speed)
    else:
        direction = "Right Turn"
        agv.clockwise_rotation(rotation_speed)
    return direction

def handle_no_line_detected(last_direction, recovery_attempts=0):
    """
    노란색 선이 검출되지 않았을 때의 처리 절차
    :param last_direction: 마지막 주행 방향
    :param recovery_attempts: 복구 시도 횟수
    :return: 업데이트된 주행 방향, 복구 시도 횟수
    """
    if recovery_attempts == 0:
        time.sleep(2)  # 2초간 대기
        return last_direction, 1  # 다음 단계로 이동
    elif recovery_attempts == 1:
        agv.stop()
        agv.clockwise_rotation(90)  # 90도 시계 방향 회전
        time.sleep(1)
        return "Searching", 2  # 회전 후 상태 변경
    elif recovery_attempts == 2:
        agv.counterclockwise_rotation(180)  # 180도 반시계 방향 회전
        time.sleep(2)
        return "Recovery", 3  # 최종 복구 시도
    else:
        agv.stop()  # 모든 시도 후 정지
        return "Stopped", 4  # 최종 상태

def main():
    """
    메인 함수: AGV의 주행 로직 실행
    """
    cap = setup_camera()                                            ##2. 카메라설정
    hsv_ranges = fixed_yellow_values()
    roi_x, roi_y, roi_width, roi_height = 240, 620, 800, 200        ##4. ROI설정
    last_direction = "Straight"
    recovery_attempts = 0
    
    while True:
        result, success = process_frame(cap, roi_x, roi_y, roi_width, roi_height, hsv_ranges)
        if not success:
            break

        left_pixels, right_pixels = count_pixels(result)

        if left_pixels == 0 and right_pixels == 0:
            last_direction, recovery_attempts = handle_no_line_detected(last_direction, recovery_attempts)
        else:
            last_direction = drive_agv(left_pixels, right_pixels, last_direction)
            recovery_attempts = 0  # 라인이 검출되면 복구 시도 횟수 리셋

        cv2.imshow('AGV Line Tracing View', result)
        if cv2.waitKey(1) & 0xFF == ord('q'):   ##5. waitkey(숫자) 설정
            break

    cap.release()
    cv2.destroyAllWindows()
    agv.disconnect()

if __name__ == "__main__":
    main()
