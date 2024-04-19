#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import String

def thresholding(img):
    imgHsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([40, 255, 255])
    mask = cv2.inRange(imgHsv, lower_yellow, upper_yellow)
    return mask

def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    matrix = cv2.getPerspectiveTransform(pts2, pts1) if inv else cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

def nothing(a):
    pass

def initializeTrackbars(intialTracbarVals, wT=480, hT=240):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0],wT//2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2],wT//2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)

def valTrackbars(wT=480, hT=240):
    widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
    heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
    widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
    heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
    points = np.float32([(widthTop, heightTop), (wT-widthTop, heightTop),
                      (widthBottom , heightBottom ), (wT-widthBottom, heightBottom)])
    return points

def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])),10,(0,0,255),-1)

    return img

def getHistogram(img, minPer=0.1, region=1):
    if region == 1:
        histValues = np.sum(img, axis=0)
    else:
        histValues = np.sum(img[img.shape[0]//region:, :], axis=0)

    maxValue = np.max(histValues)
    minValue = minPer * maxValue

    indexArray = np.where(histValues >= minValue)
    basePoint = int(np.average(indexArray)) #if indexArray[0].size > 0 else img.shape[1]//2

    imgHist =np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    for x, intensity in enumerate(histValues):
        cv2.line(imgHist, (x, img.shape[0]), (x, int(img.shape[0] - intensity//255//region)), (255,0,255),1)
        cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0,255,255), cv2.FILLED)

    return basePoint, imgHist
    
    # imgHist = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)
    # for x, intensity in enumerate(histValues):
    #     if intensity > 0:
    #         scaled_intensity = int(intensity / 255 / region)
    #         end_point = max(0, img.shape[0] - scaled_intensity)
    #         cv2.line(imgHist, (x, img.shape[0]), (x, end_point), (255, 0, 255), 1)
    # cv2.circle(imgHist, (basePoint, img.shape[0]), 20, (0, 255, 255), cv2.FILLED)

    

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver

curveList = []
avgVal = 10

def getLaneCurve(img):
    
    imgCopy = img.copy()
    imgResult = img.copy()

    # step1
    imgThres = thresholding(img)

    # step2
    #ROI영역 <- Calibratioin
    hT, wT, c = img.shape
    intialTracBarVals = [40, 240, 60, 180]
    points = np.float32([(40, 220), (440, 220),
                      (60 , 130 ), (420, 130)])
    imgWarp = warpImg(imgThres, points, wT, hT)
    imgWarpPoints = drawPoints(imgCopy, points)

    # step 3
    middlePoint, imgHist = getHistogram(imgWarp, minPer=0.5, region=4)
    curveAveragePoint, imgHist = getHistogram(imgWarp, minPer=0.9)
    curveRaw = curveAveragePoint - middlePoint

    # step 4
    curveList.append(curveRaw)
    if len(curveList) > avgVal:
        curveList.pop(0)
    
    curve = int(sum(curveList)/len(curveList))

    imgInvWarp = warpImg(imgWarp, points, wT, hT,inv = True)
    imgInvWarp = cv2.cvtColor(imgInvWarp,cv2.COLOR_GRAY2BGR)
    imgInvWarp[0:hT//3,0:wT] = 0,0,0
    imgLaneColor = np.zeros_like(img)
    imgLaneColor[:] = 0, 255, 0
    imgLaneColor = cv2.bitwise_and(imgInvWarp, imgLaneColor)
    imgResult = cv2.addWeighted(imgResult,1,imgLaneColor,1,0)
    midY = 450
    cv2.putText(imgResult,str(curve),(wT//2-80,85),cv2.FONT_HERSHEY_COMPLEX,2,(255,0,255),3)
    cv2.line(imgResult,(wT//2,midY),(wT//2+(curve*3),midY),(255,0,255),5)
    cv2.line(imgResult, ((wT // 2 + (curve * 3)), midY-25), (wT // 2 + (curve * 3), midY+25), (0, 255, 0), 5)
    for x in range(-30, 30):
        w = wT // 20
        cv2.line(imgResult, (w * x + int(curve//50 ), midY-10),(w * x + int(curve//50 ), midY+10), (0, 0, 255), 2)

    imgStacked = stackImages(0.7,([img,imgWarpPoints,imgWarp],[imgHist,imgLaneColor,imgResult]))

    # NORMALIZATION
    curve = curve/100
    if curve > 1: curve ==1
    if curve < -1 : curve == -1


    return curve, imgStacked



def process_image(data):
    
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        img_cv = cv2.resize(cv_image, (480, 240))
    
        curve, imgStacked = getLaneCurve(img_cv) 

        cv2.imshow('ImageStack',imgStacked)
        
        #다음 계산까지 대기 <- Calibration
        cv2.waitKey(100)#Calibration


    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    print(f'curve: {curve}')


    #회전 기준 <-Calibration
    if curve <= -0.07:
        command = "Forward_L"
    elif curve >= 0.05:
        command = "Forward_R"
    else:
        command = "Forward"

    pub = rospy.Publisher('control_commands', String, queue_size=1)
    pub.publish(command)

if __name__ == '__main__':
    rospy.init_node('image_processor', anonymous=True)
    rospy.Subscriber('video_frames', Image, process_image)
    고