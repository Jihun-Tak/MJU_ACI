import psycopg2
from datetime import datetime



def get_data_and_average(month, dow, hour, dong_index):
    # PostgreSQL 연결 정보 설정
    connection = psycopg2.connect(
        host="localhost",
        database="CapDe4",
        user="postgres",
        password="4444"
    )

    # PostgreSQL 쿼리 실행
    query = f"""
        SELECT city, gu, dong, dong_index, r_year, r_month, r_day, r_dofw, r_hour, us_rate
        FROM usage_rate
        WHERE r_month = {month} AND r_dofw = {dow} AND r_hour = {hour} AND dong_index = {dong_index}
    """

    with connection.cursor() as cursor:
        cursor.execute(query)
        data = cursor.fetchall()

    # 데이터 출력
    for row in data:
        city, gu, dong, dong_index, r_year, r_month, r_day, r_dofw, r_hour, us_rate = row
        print(f"City: {city}, Gu: {gu}, Dong: {dong}, Dong Index: {dong_index}, "
              f"Year: {r_year}, Month: {r_month}, Day: {r_day}, Day of Week: {r_dofw}, "
              f"Hour: {r_hour}, Usage Rate: {us_rate}")

    # 사용율 평균 계산 및 출력
    if data:
        average_usage_rate = sum(row[-1] for row in data) / len(data)
        print(f"Average Usage Rate: {average_usage_rate:.2f}")

    # 연결 종료
    connection.close()

# 현재 시간 정보를 얻어옴
now = datetime.now()
current_month = now.month
current_day_of_week = now.weekday()  # 월요일: 0, 일요일: 6 (요일 순서를 변경하여 저장)
current_hour = now.hour

# 토요일은 6으로 저장되므로, 일요일은 0으로, 토요일은 6으로 변환
if current_day_of_week == 6:
    current_day_of_week = 0
else:
    current_day_of_week += 1

# 현재 시간 정보를 함수에 전달
print("현재 ->",current_month,"월", current_day_of_week, current_hour,"시간 대")
# 예시 데이터 입력 (월, 요일, 시간대, 동번호)
#get_data_and_average(current_month, current_day_of_week, current_hour, 44)
get_data_and_average(7, 0, 18, 44)
