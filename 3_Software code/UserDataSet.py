import psycopg2  #https://studying-haeung.tistory.com/18

# PostgreSQL 연결 정보 설정
connection = psycopg2.connect(
    host="localhost",
    database="CapDe4",
    user="postgres",
    password="4444"
)

# 커서 생성
cursor = connection.cursor()

# SQL 쿼리 작성
sql_query = "SELECT user_phone, car_licenseplate, car_model, car_portlocation FROM user_infor WHERE car_licenseplate = '404명0118'"

# 쿼리 실행
cursor.execute(sql_query)

# 결과 가져오기
row = cursor.fetchone()

# 데이터 할당
if row:
    phone, carnum, model, port = row
else:
    print("해당하는 데이터를 찾을 수 없습니다.")

# 연결과 커서 닫기
cursor.close()
connection.close()

# 변수 확인
print("Phone:", phone)
print("Car Number:", carnum)
print("Car Model:", model)
print("Car Port Location:", port)
