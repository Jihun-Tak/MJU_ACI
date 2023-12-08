import psycopg2

# PostgreSQL 연결 정보 설정
connection = psycopg2.connect(
    host="localhost",
    database="CapDe4",
    user="postgres",
    password="4444"
)

# 커서 생성
cursor = connection.cursor()

# 데이터 삽입 SQL 쿼리 작성
insert_query = """
INSERT INTO user_infor (user_phone, car_licenseplate, car_model, car_portlocation)
VALUES (%s, %s, %s, %s)
"""

# 데이터
user_phone = '01012345678'
car_licenseplate = '100바7777'
car_model = 'Benz [SL65]'
car_portlocation = 3

# 데이터 삽입
cursor.execute(insert_query, (user_phone, car_licenseplate, car_model, car_portlocation))

# 변경 사항을 데이터베이스에 반영
connection.commit()

# 연결과 커서 닫기
cursor.close()
connection.close()

print("데이터가 성공적으로 삽입되었습니다.")
