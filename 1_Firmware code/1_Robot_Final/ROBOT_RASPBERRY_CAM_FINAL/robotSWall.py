import cv2
import numpy as np
import copy
import paho.mqtt.client as mqtt
import time
import spidev
import sys

SystemAllFlag = 0
frame = None
t = None
t_cir = None
f = None
b = None
gray = None
gray_cir = None
binary = None
binary_cir = None
Circle = None
cnt_x = 0
cnt_y = 0
cnt_x_cir = 0
cnt_y_cir = 0
parallel = 0
flagMQTT = 0
Zflag = 0
XFlag=0
sublight = 0
light = 60
YYY = 0
ZZZ = 0
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

def readChannel(channel):
    val = spi.xfer2([1,(8+channel)<<4,0])
    data = ((val[1]&3)<<8)+val[2]
    return data

def on_message(client, userdata, message): #MQTT콜백
    global RobotStep, light,YYY, ZZZ,SystemAllFlag

    print("message received : ", str(message.payload.decode("utf-8")))
    print("message topic= ", message.topic)
    print("message qos= ", message.qos)
    print("message retain flag= ", message.retain)
    if message.topic == "MJU/CD4/CHARGING":
        if str(message.payload.decode("utf-8")) == "START":
            RobotStep = 1
            print("success")

    if message.topic == "MJU/CD4/CHARGING/LIGHT":
        if str(message.payload.decode("utf-8")) == "-":
            light = light - 10
            mqtt.publish("MJU/CD4/CHARGING/LIGHT",light)
        elif str(message.payload.decode("utf-8")) == "+":
            light = light + 10
            mqtt.publish("MJU/CD4/CHARGING/LIGHT",light)
        elif str(message.payload.decode("utf-8")) == "START":
            YYY = 1
        elif str(message.payload.decode("utf-8")) == "STOP":
            YYY = 0
            ZZZ = 0

    if message.topic == "MJU/CD4/CHARGING/CAR":
        if str(message.payload.decode("utf-8")) == "IN": #차량 진입->해당위치로 이동 단계
            SystemAllFlag = 1
        elif str(message.payload.decode("utf-8")) == "FindCARNUM": #번호판 인식단계
            SystemAllFlag = 2
        elif str(message.payload.decode("utf-8")) == "FindConnector": #커넥터 찾아서 잡는 단계
            SystemAllFlag = 3
        elif str(message.payload.decode("utf-8")) == "GoToPort": #차량 충전구로 이동 단계
            SystemAllFlag = 4
        elif str(message.payload.decode("utf-8")) == "FindPort": #충전구 탐색 및 삽입 단계
            SystemAllFlag = 5
        elif str(message.payload.decode("utf-8")) == "Success": #충전연결 성공, 커넥터 꽂은채로 로봇팔 회수 단계
            SystemAllFlag = 6
        elif str(message.payload.decode("utf-8")) == "BackForWaiting": #로봇 원래자리로 돌아가는 단계
            SystemAllFlag = 7
        elif str(message.payload.decode("utf-8")) == "WaitingCharge": #충전완료까지 대기하는 단계
            SystemAllFlag = 8
        elif str(message.payload.decode("utf-8")) == "StopCharging": #충전정지신호 받고 로봇 충전구로 이동하는 단계
            SystemAllFlag = 9
        elif str(message.payload.decode("utf-8")) == "PullConnector": #커넥터 찾아서 잡고 전자서 켜서 뽑는 단계
            SystemAllFlag = 10
        elif str(message.payload.decode("utf-8")) == "PutConnector": #커넥터 돌려놓으면서 회수시스템 작동 단계
            SystemAllFlag = 11
        elif str(message.payload.decode("utf-8")) == "CARINWaiting": #원점으로 돌아가는 단계 끝나면 SystemAllFlag = 0으로 돌아가서 반복
            SystemAllFlag = 12


def center_xy_Circle(flag):
    global frame, gray_cir, binary_cir, cnt_x_cir, cnt_y_cir, Circle, light

    gray_cir = cv2.cvtColor(Circle, cv2.COLOR_BGR2GRAY)

    print("--------------------------")
    print(gray_cir.mean())
    
    #if(abs(140 - gray_cir.mean()) > 0):
    #    gray_cir = gray_cir + abs(140 - int(gray_cir.mean()))

    t_cir = cv2.bilateralFilter(gray_cir, -1, 10, 5)
    _, binary_cir = cv2.threshold(t_cir, light, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(binary_cir, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if(flag == 1):
        
        first = 0
        last = 0
        cnt = 0
        centers = []
        radius = np.array([])  # Initialize radius as an empty NumPy array
        for contour in contours:
            if len(contour) < 5:
                continue

            area = cv2.contourArea(contour)
            if area < 1 or area > 500:
                continue

            (x, y), rad = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = np.append(radius, rad) 

            cv2.circle(Circle, center, int(rad), (0, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(Circle, center, 10, (0, 0, 255), -1, cv2.LINE_AA)

            centers.append(center)

        if len(centers) >0:
            cnt_x_cir = 0
            cnt_y_cir = 0
            for center in centers:
                cnt_x_cir += centers[0][0]
                cnt_y_cir += centers[0][1]
                cv2.circle(Circle, center, 10, (0, 255, 0), -1, cv2.LINE_AA)
            cnt_x_cir /= len(centers)
            cnt_y_cir /= len(centers)


def center_xy_Rect(flag):
    global frame, gray, binary, cnt_x, cnt_y, frame_x, frame_y,light

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    print("--------------------------")
    print(gray.mean())
    
    #if(abs(140 - gray.mean()) > 0):
    #    gray = gray + abs(140 - int(gray.mean()))

    if(flag == 1):
        t = cv2.bilateralFilter(gray, -1, 10, 5)
        _, binary = cv2.threshold(t, light, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #윤곽선 검출

        centers = []
        for contour in contours:
            if len(contour) < 4:  
                continue

            area = cv2.contourArea(contour)
            if (area < 3000): #면적 계산, 500보다 작거나 13000보다 크면 윤곽무시
                continue
            
            if (70000 < area): #면적 계산, 500보다 작거나 13000보다 크면 윤곽무시
                continue
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)  #다각형 근사

            if len(approx) == 4:  
                (x, y, w, h) = cv2.boundingRect(approx)
                center = (int(x + w/2), int(y + h/2))
                centers.append(center)

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2) #바운딩 박스그리기
                cv2.circle(frame, center, 10, (0, 0, 255), -1) #중심좌표에 원 그리기

        if len(centers) == 1: 
            (x, y) = centers[0]
            cnt_x = x
            cnt_y = y
            cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)
            #cnt_x /= len(centers)
            #cnt_y /= len(centers)

            print(f"검출완료 X좌표: {cnt_x}, Y좌표: {cnt_y}")



def move_center():
    global frame, cnt_x, cnt_y, flagMQTT, Circle, mqtt, Zflag, YYY, ZZZ

    print("중심으로 이동중....")
    flagMQTT += 1
    if (flagMQTT > 2):
        if(Zflag == 0):
            if(YYY == 1):
                mqtt.publish("MJU/IOT/60181793/Y", cnt_x) #토픽과 메세지 발행
            
            if(ZZZ == 1):
                mqtt.publish("MJU/IOT/60181793/Z", cnt_y) #토픽과 메세지 발행
            flagMQTT = 0


#def move_center_Y():
 #   global frame, cnt_x, cnt_y, flagMQTT, Circle, mqtt, Zflag
#
 #   print("중심으로 이동중....")
  #  flagMQTT += 1
   # if (flagMQTT > 2):
    #    if(Zflag == 0):

  


def insertion():
    arm_motor()


def stage_motor(y):
    print("y축 이동중....")


def wheel_motor(x):
    print("x축 이동중....")


def arm_motor():
  
    print("팔 뻗는중....")

#def on_connect(client, userdata, flags, rc):
#	print("Connected with result code "+str(rc))
	# Subscribing in on_connect() means that if we lose the connection and
	# reconnect then subscriptions will be renewed.
#	client.subscribe("MJU/IOT/60181793/Flag")


# The callback for when a PUBLISH message is received from the server.
#def on_message(client, userdata, msg):
#    global flagMQTT
#    print(str(msg.payload))
#    flagMQTT = str(msg.payload)
def motiondetect(im1,im2,im3):
    global flag, gray_img01, gray_img02, gray_img03 , ret
    gray_img01 = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    gray_img02 = cv2.cvtColor(im2, cv2.COLOR_BGR2GRAY)
    gray_img03 = cv2.cvtColor(im3, cv2.COLOR_BGR2GRAY)
    diff01 = cv2.absdiff(gray_img01,gray_img02)
    diff02 = cv2.absdiff(gray_img02,gray_img03)

    ret, diff01 = cv2.threshold(diff01, 20, 255, cv2.THRESH_BINARY)
    ret, diff02 = cv2.threshold(diff02, 20, 255, cv2.THRESH_BINARY)
    
    diff = cv2.bitwise_and(diff01, diff02)
    diff_cnt = cv2.countNonZero(diff) # amount of difference(the closer to zero, the less differeceq)

    #cv2.imshow("Motion sensing", diff)
    print(diff_cnt)

    if(diff_cnt > 20): flag = 0
    elif(diff_cnt <= 20): flag = 1

def recognize_license_plate(frame1): #번호판
    # 그레이스케일로 변환
    final = frame1

    gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    thresholded = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 19, 9)

    contours, _ = cv2.findContours(thresholded, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    temp_result = np.zeros(frame1.shape, dtype=np.uint8)

    cv2.drawContours(temp_result, contours=contours, contourIdx=-1, color=(255, 255, 255))

    temp_result2 = np.zeros(frame1.shape, dtype=np.uint8)

    contours_dict = []   #윤곽선 주변의 사각형 경계상자

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(temp_result2, pt1=(x, y), pt2=(x + w, y + h), color=(255, 255, 255), thickness=2)
        contours_dict.append({
            'contour': contour,
            'x': x,
            'y': y,
            'w': w,
            'h': h,
            'cx': x + (w / 2),
            'cy': y + (h / 2)
        })

    MIN_AREA = 80   #윤곽선의 최소 넓이
    MIN_WIDTH, MIN_HEIGHT = 2, 8 #윤곽선의 최소 너비와 높이
    MIN_RATIO, MAX_RATIO = 0.25, 1.0  #윤곽선의 종횡비 (너비/높이) 범위

    possible_contours = []
    cnt = 0

    for d in contours_dict:
        area = d['w'] * d['h']
        ratio = d['w'] / d['h']

        if area > MIN_AREA and d['w'] > MIN_WIDTH and d['h'] > MIN_HEIGHT and MIN_RATIO < ratio < MAX_RATIO:
            d['idx'] = cnt
            cnt += 1
            possible_contours.append(d)

    temp_result3 = np.zeros(frame1.shape, dtype=np.uint8)

    for d in possible_contours:
        cv2.rectangle(temp_result3, pt1=(d['x'], d['y']), pt2=(d['x'] + d['w'], d['y'] + d['h']), color=(255, 255, 255), thickness=2)

    MAX_DIAG_MULTIPLYER = 5
    MAX_ANGLE_DIFF = 12.0
    MAX_AREA_DIFF = 0.5
    MAX_WIDTH_DIFF = 0.8
    MAX_HEIGHT_DIFF = 0.2
    MIN_N_MATCHED = 3

    def find_chars(contour_list):  #문자일 가능성이 있는 윤곽선
        matched_result_idx = []

        for d1 in contour_list:
            matched_contours_idx = []
            for d2 in contour_list:
                if d1['idx'] == d2['idx']:
                    continue

                dx = abs(d1['cx'] - d2['cx'])
                dy = abs(d1['cy'] - d2['cy'])
                diagonal_length1 = np.sqrt(d1['w'] ** 2 + d1['h'] ** 2)

                distance = np.linalg.norm(np.array([d1['cx'], d1['cy']]) - np.array([d2['cx'], d2['cy']]))

                if dx == 0:
                    angle_diff = 90
                else:
                    angle_diff = np.degrees(np.arctan(dy / dx))

                area_diff = abs(d1['w'] * d1['h'] - d2['w'] * d2['h']) / (d1['w'] * d1['h'])
                width_diff = abs(d1['w'] - d2['w']) / d1['w']
                height_diff = abs(d1['h'] - d2['h']) / d1['h']

                if distance < diagonal_length1 * MAX_DIAG_MULTIPLYER and angle_diff < MAX_ANGLE_DIFF and area_diff < MAX_AREA_DIFF and width_diff < MAX_WIDTH_DIFF and height_diff < MAX_HEIGHT_DIFF:
                    matched_contours_idx.append(d2['idx'])

            matched_contours_idx.append(d1['idx'])

            if len(matched_contours_idx) < MIN_N_MATCHED:
                continue

            matched_result_idx.append(matched_contours_idx)

            unmatched_contour_idx = []
            for d4 in contour_list:
                if d4['idx'] not in matched_contours_idx:
                    unmatched_contour_idx.append(d4['idx'])

            unmatched_contour = np.take(possible_contours, unmatched_contour_idx)

            recursive_contour_list = find_chars(unmatched_contour)

            for idx in recursive_contour_list:
                matched_result_idx.append(idx)

            break

        return matched_result_idx

    result_idx = find_chars(possible_contours)

    matched_result = []  
    for idx_list in result_idx:
        matched_result.append(np.take(possible_contours, idx_list))

    temp_result4 = np.zeros(frame1.shape, dtype=np.uint8)  # 문자일 가능성이 있는 윤곽선에 사각형 경계상자

    for r in matched_result:
        for d in r:
            cv2.rectangle(final, pt1=(d['x'], d['y']), pt2=(d['x'] + d['w'], d['y'] + d['h']), color=(0, 0, 255), thickness=2)

    PLATE_WIDTH_PADDING = 1.3
    PLATE_HEIGHT_PADDING = 1.5
    MIN_PLATE_RATIO = 3
    MAX_PLATE_RATIO = 10

    plate_imgs = []
    plate_infos = []

    for i, matched_chars in enumerate(matched_result): #문자 다시 한번 더 순회
        sorted_chars = sorted(matched_chars, key=lambda x: x['cx'])

        plate_cx = (sorted_chars[0]['cx'] + sorted_chars[-1]['cx']) / 2
        plate_cy = (sorted_chars[0]['cy'] + sorted_chars[-1]['cy']) / 2

        plate_width = (sorted_chars[-1]['x'] + sorted_chars[-1]['w'] - sorted_chars[0]['x']) * PLATE_WIDTH_PADDING

        sum_height = 0
        for d in sorted_chars:
            sum_height += d['h']

        plate_height = int(sum_height / len(sorted_chars) * PLATE_HEIGHT_PADDING)

        triangle_height = sorted_chars[-1]['cy'] - sorted_chars[0]['cy']
        triangle_hypotenus = np.linalg.norm(np.array([sorted_chars[0]['cx'], sorted_chars[0]['cy']]) - np.array([sorted_chars[-1]['cx'], sorted_chars[-1]['cy']]))

        angle = np.degrees(np.arcsin(triangle_height / triangle_hypotenus))

        rotation_matrix = cv2.getRotationMatrix2D(center=(plate_cx, plate_cy), angle=angle, scale=1.0) #삐뚤어진 번호판들 정렬
     
        img_rotated = cv2.warpAffine(thresholded, M=rotation_matrix, dsize=(frame1.shape[1], frame1.shape[0]))

        img_cropped = cv2.getRectSubPix(
            img_rotated,
            patchSize=(int(plate_width), int(plate_height)),
            center=(int(plate_cx), int(plate_cy))
        )

        if img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO or img_cropped.shape[1] / img_cropped.shape[0] < MIN_PLATE_RATIO > MAX_PLATE_RATIO:
            continue

        plate_imgs.append(img_cropped)
        plate_infos.append({
            'x': int(plate_cx - plate_width / 2),
            'y': int(plate_cy - plate_height / 2),
            'w': int(plate_width),
            'h': int(plate_height)
        })

    longest_idx, longest_text = -1, 0
    plate_chars = []

    for i, plate_img in enumerate(plate_imgs):
        plate_img = cv2.resize(plate_img, dsize=(0, 0), fx=1.6, fy=1.6)
        _, plate_img = cv2.threshold(plate_img, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)

        contours, _ = cv2.findContours(plate_img, mode=cv2.RETR_LIST, method=cv2.CHAIN_APPROX_SIMPLE)

        plate_min_x, plate_min_y = plate_img.shape[1], plate_img.shape[0]
        plate_max_x, plate_max_y = 0, 0

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = w * h
            ratio = w / h

            if area > MIN_AREA and w > MIN_WIDTH and h > MIN_HEIGHT and MIN_RATIO < ratio < MAX_RATIO:
                if x < plate_min_x:
                    plate_min_x = x
                if y < plate_min_y:
                    plate_min_y = y
                if x + w > plate_max_x:
                    plate_max_x = x + w
                if y + h > plate_max_y:
                    plate_max_y = y + h

        img_result = plate_img[plate_min_y:plate_max_y, plate_min_x:plate_max_x]

        img_result = cv2.GaussianBlur(img_result, ksize=(3, 3), sigmaX=0)
        _, img_result = cv2.threshold(img_result, thresh=0.0, maxval=255.0, type=cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        img_result = cv2.copyMakeBorder(img_result, top=10, bottom=10, left=10, right=10, borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0))

    return final


def main():
    global frame, cnt_x, cnt_y, x, y, parallel, Circle, mqtt,flag, ret, img01, img02, img03, Zflag, Xflag, YYY, ZZZ, SystemAllFlag
    flagMQTT = 0

    mqtt = mqtt.Client("python_pub") #Mqtt Client 오브젝트 생성
    mqtt.connect("cheese.mju.ac.kr", 30220) #MQTT 서버에 연결
    mqtt.subscribe("MJU/CD4/CHARGING") #차량 진입 시
    mqtt.subscribe("MJU/CD4/CHARGING/LIGHT")
    mqtt.subscribe("MJU/CD4/CHARGING/CAR")
    mqtt.on_message = on_message
    mqtt.loop_start()
    
    

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open the camera.")
        return

    cap.set(cv2.CAP_PROP_FPS, 8)
    cv2.namedWindow("Real-time video", cv2.WINDOW_NORMAL)
    cv2.setWindowProperty("Real-time video", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN) #전체화면
    _, frame = cap.read()
    frame_y = frame.shape[0] / 2  
    frame_x = frame.shape[1] / 2

    while True:

        _, frame = cap.read()
        ret, img01 = cap.read()
        ret, img02 = cap.read()
        ret, img03 = cap.read()
        Circle = copy.deepcopy(frame)
        frame1 = copy.deepcopy(frame)

        if frame is None:
            print("Could not read the frame.")
            break
        #time.sleep(1)
        # Start

        if SystemAllFlag == 2:
            frame = recognize_license_plate(frame1)
            mqtt.publish("MJU/CD4/CHARGING/CAR","FindConnector")

        elif SystemAllFlag == 3 or SystemAllFlag == 5 or SystemAllFlag == 10:
            motiondetect(img01,img02,img03)
            center_xy_Rect(flag)
            center_xy_Circle(flag)

            if flag == 1 :
                print(frame_x)
                print(frame_y)
                print(cnt_x)
                print(cnt_y)
                #MQTT(cnt_x,cnt_y)

                size = 50
                thickness = 2
                pt1 = (int(frame_x - size / 2), int(frame_y))
                pt2 = (int(frame_x + size / 2), int(frame_y))
                pt3 = (int(frame_x), int(frame_y - size / 2))
                pt4 = (int(frame_x), int(frame_y + size / 2))

                if abs(frame_y - cnt_y) < 4 and cnt_y > 0:
                    if parallel == 0:
                        if Zflag == 0:
                            mqtt.publish("MJU/IOT/60181793/Z","9999") #토픽과 메세지 발행
                            ZZZ = 0
                            YYY = 1
                            time.sleep(3)
                            mqtt.publish("MJU/IOT/60181793/X","120")
                            time.sleep(3) 
                            mqtt.publish("MJU/IOT/60181793/X","60")               #        Zflag = 1
                    #    move_slope()
                    #    cv2.line(Circle, pt1, pt2, (255, 0, 0), thickness)
                    #    cv2.line(Circle, pt3, pt4, (255, 0, 0), thickness)
                    #if parallel == 1:
                    #    insertion()
                    #    cv2.line(Circle, pt1, pt2, (0, 255, 255), thickness)
                    #    cv2.line(Circle, pt3, pt4, (0, 255, 255), thickness)
                elif cnt_y > 0:
                    cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
                    cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                    move_center()

                if abs(frame_x+34 - cnt_x) < 1 and cnt_x > 0 :
                    if parallel == 0:
                        if Zflag == 0:
                            mqtt.publish("MJU/IOT/60181793/Y","99999") #토픽과 메세지 발행
                            ZZZ = 1
                            YYY = 0
                    #       Zflag = 1
                    #    move_slope()
                    #    cv2.line(Circle, pt1, pt2, (255, 0, 0), thickness)
                    #    cv2.line(Circle, pt3, pt4, (255, 0, 0), thickness)
                    #if parallel == 1:
                    #    insertion()
                    #    cv2.line(Circle, pt1, pt2, (0, 255, 255), thickness)
                    #    cv2.line(Circle, pt3, pt4, (0, 255, 255), thickness)
                elif cnt_x > 0 :
                    cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
                    cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                    move_center()
                
                

                    #Zflag = 0
                # End
                #elif cnt_x > 0:
                    #cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
                    #cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                    #move_center_X()
                    #if abs(frame_x - cnt_x) < 3:
                    #    XFlag = 1

            # elif cnt_y > 0 and XFlag == 1:
            #     cv2.line(Circle, pt1, pt2, (0, 255, 0), thickness)
            #     cv2.line(Circle, pt3, pt4, (0, 255, 0), thickness)
                #    move_center_Y()
                    #Zflag = 0
                # End

        elif SystemAllFlag == 4 or SystemAllFlag == 6 or SystemAllFlag == 7 or SystemAllFlag == 8 or SystemAllFlag == 9 or SystemAllFlag == 11:
            print("Resting...")

        elif SystemAllFlag == 12:
            SystemAllFlag = 0

        cv2.imshow("Real-time video", frame)
        #cv2.imshow("threshold", binary_cir)
        #cv2.imshow("circle",Circle)
            #time.sleep(0.5)
        if cv2.waitKey(1) == 27:
                break

        print("\n" * 10)  # Clear the console
            #client.loop_forever()
            #client.loop()
            #cv2.waitKey(10)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()