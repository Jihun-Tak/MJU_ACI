import psycopg2
import time
import copy
import paho.mqtt.client as mqtt
import sys

sys.path.append('C:/Users/sitepackages')
sys.path.append('C:/Users/sitepackages')

# PostgreSQL 연결 정보 설정
connection = psycopg2.connect(
    host="localhost",
    database="CapDe4",
    user="postgres",
    password="4444"
    
)

global user_phone,car_licenseplate,car_model,car_portlocation

def insert_data():
    global user_phone,car_licenseplate,car_model,car_portlocation
    # 커서 생성
    cursor = connection.cursor()

    # 데이터 삽입 SQL 쿼리 작성
    insert_query = """
    INSERT INTO user_infor (user_phone, car_licenseplate, car_model, car_portlocation)
    VALUES (%s, %s, %s, %s)
    """

    # 데이터 삽입
    cursor.execute(insert_query, (user_phone, car_licenseplate, car_model, car_portlocation))

    # 변경 사항을 데이터베이스에 반영
    connection.commit()

    # 연결과 커서 닫기
    cursor.close()
    connection.close()

    print("데이터가 성공적으로 삽입되었습니다.")


def on_message(client, userdata, message):
    global user_phone,car_licenseplate,car_model,car_portlocation
    
    print("message received ", str(message.payload.decode("utf-8")))
    print("message topic=", message.topic)
    print("message qos=", message.qos)
    print("message retain flag=", message.retain)

    MQtopic = message.topic

    if MQtopic == "MJU/CD4/NEW/PHONE":
        user_phone = str(message.payload.decode("utf-8"))
    elif MQtopic == "MJU/CD4/NEW/CARNUM":
        car_licenseplate = str(message.payload.decode("utf-8"))
    elif MQtopic == "MJU/CD4/NEW/MODEL":
        car_model = str(message.payload.decode("utf-8"))
    elif MQtopic == "MJU/CD4/NEW/CHARGEPORT":
        car_portlocation = str(message.payload.decode("utf-8"))
        insert_data()

    




def main():
    global user_phone,car_licenseplate,car_model,car_portlocation

    flagMQTT = 0

    client1 = mqtt.Client("python_pub") # Mqtt Client 오브젝트 생성
    client1.connect("cheese.mju.ac.kr", 30220) # MQTT 서버에 연결
    client1.subscribe("MJU/CD4/NEW/PHONE")
    client1.subscribe("MJU/CD4/NEW/MODEL")
    client1.subscribe("MJU/CD4/NEW/CARNUM")
    client1.subscribe("MJU/CD4/NEW/CHARGEPORT")

    client1.on_message = on_message

    client1.loop_forever()





if __name__ == "__main__":
    main()


# 데이터
#user_phone = '01012345678'
#car_licenseplate = '100바7777'
#car_model = 'Benz [SL65]'
#car_portlocation = 3


