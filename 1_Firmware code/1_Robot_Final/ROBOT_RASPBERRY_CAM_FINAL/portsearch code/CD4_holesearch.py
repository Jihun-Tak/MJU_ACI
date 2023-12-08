import cv2
import numpy as np
import time
import copy
import paho.mqtt.client as mqtt
import time
import spidev


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
spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz=1000000

def readChannel(channel):
    val = spi.xfer2([1,(8+channel)<<4,0])
    data = ((val[1]&3)<<8)+val[2]
    return data


def center_xy_Circle(flag):
    global frame, gray_cir, binary_cir, cnt_x_cir, cnt_y_cir, Circle

    gray_cir = cv2.cvtColor(Circle, cv2.COLOR_BGR2GRAY)

    print("--------------------------")
    print(gray_cir.mean())
    
    if(abs(140 - gray_cir.mean()) > 0):
        gray_cir = gray_cir + abs(140 - int(gray_cir.mean()))

    t_cir = cv2.bilateralFilter(gray_cir, -1, 10, 5)
    _, binary_cir = cv2.threshold(t_cir, 9, 255, cv2.THRESH_BINARY)
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
    global frame, gray, binary, cnt_x, cnt_y, frame_x, frame_y

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    print("--------------------------")
    print(gray.mean())
    
    if(abs(140 - gray.mean()) > 0):
        gray = gray + abs(140 - int(gray.mean()))

    if(flag == 1):
        t = cv2.bilateralFilter(gray, -1, 10, 5)
        _, binary = cv2.threshold(t, 80, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) #윤곽선 검출

        centers = []
        for contour in contours:
            if len(contour) < 4:  
                continue

            area = cv2.contourArea(contour)
            if (area < 3000): #면적 계산, 500보다 작거나 13000보다 크면 윤곽무시
                continue
            
            if (140000 < area): #면적 계산, 500보다 작거나 13000보다 크면 윤곽무시
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
    global frame, cnt_x, cnt_y, flagMQTT, Circle, mqtt, Zflag

    print("중심으로 이동중....")
    flagMQTT += 1
    if (flagMQTT > 2):
        if(Zflag == 0):
            mqtt.publish("MJU/IOT/60181793/Y", cnt_x) #토픽과 메세지 발행
            mqtt.publish("MJU/IOT/60181793/Z", cnt_y) #토픽과 메세지 발행
            flagMQTT = 0


#def move_center_Y():
 #   global frame, cnt_x, cnt_y, flagMQTT, Circle, mqtt, Zflag
#
 #   print("중심으로 이동중....")
  #  flagMQTT += 1
   # if (flagMQTT > 2):
    #    if(Zflag == 0):

           



def move_slope():
    global flagMQTT, mqtt, parallel, Zflag
    slope = 0.0
    print("기울기 조정중....")
    v1=(readChannel(0)/1023.0)*3.3
    v2=(readChannel(1)/1023.0)*3.3
    dist1 = 16.2537*v1**4-129.893 * v1**3 + 382.268 * v1**2 - 512.611 * v1 + 301.439
    dist2 = 16.2537*v2**4-129.893 * v2**3 + 382.268 * v2**2 - 512.611 * v2 + 301.439
        
    if (dist1 - dist2) > 0.5 : 
        flagMQTT += 1
        if (flagMQTT > 2):
            mqtt.publish("MJU/IOT/60181793/Slope", "UP") #토픽과 메세지 발행
            flagMQTT = 0

    elif (dist1 - dist2) < -0.5 :
        flagMQTT += 1
        if (flagMQTT > 2):
            mqtt.publish("MJU/IOT/60181793/Slope", "DOWN") #토픽과 메세지 발행
            flagMQTT = 0

    else :
        flagMQTT += 1
        if (flagMQTT > 2):
            mqtt.publish("MJU/IOT/60181793/Slope", "OK") #토픽과 메세지 발행
            flagMQTT = 0
            parallel = 1
            #Zflag = 0

  


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

    cv2.imshow("Motion sensing", diff)
    print(diff_cnt)

    if(diff_cnt > 5): flag = 0
    elif(diff_cnt <= 5): flag = 1



def main():
    global frame, cnt_x, cnt_y, x, y, parallel, Circle, mqtt,flag, ret, img01, img02, img03, Zflag, Xflag
    flagMQTT = 0
    #client = mqtt.Client()
    #client.on_connect = on_connect
    #client.on_message = on_message
    #client.connect("cheese.mju.ac.kr", 30220, 60)

    mqtt = mqtt.Client("python_pub") #Mqtt Client 오브젝트 생성
    mqtt.connect("cheese.mju.ac.kr", 30220) #MQTT 서버에 연결

    

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open the camera.")
        return

    cap.set(cv2.CAP_PROP_FPS, 8)
    cv2.namedWindow("Real-time video", cv2.WINDOW_NORMAL)
    _, frame = cap.read()
    frame_y = frame.shape[0] / 2  
    frame_x = frame.shape[1] / 2

    while True:

        _, frame = cap.read()
        ret, img01 = cap.read()
        ret, img02 = cap.read()
        ret, img03 = cap.read()
        Circle = copy.deepcopy(frame)

        if frame is None:
            print("Could not read the frame.")
            break
        #time.sleep(1)
        # Start
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

            if abs(frame_x - cnt_x) < 50 and abs(frame_y - cnt_y) < 3 and cnt_x > 0 and cnt_y > 0:
                if parallel == 0:
                    if Zflag == 0:
                        mqtt.publish("MJU/IOT/60181793/Z","9999") #토픽과 메세지 발행
                        Zflag = 1
                    move_slope()
                    cv2.line(Circle, pt1, pt2, (255, 0, 0), thickness)
                    cv2.line(Circle, pt3, pt4, (255, 0, 0), thickness)
                if parallel == 1:
                    insertion()
                    cv2.line(Circle, pt1, pt2, (0, 255, 255), thickness)
                    cv2.line(Circle, pt3, pt4, (0, 255, 255), thickness)
            elif cnt_x > 0 and cnt_y > 0:
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

        cv2.imshow("Real-time video", frame)
        cv2.imshow("threshold", binary_cir)
        cv2.imshow("circle",Circle)
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