import mysql.connector
from mysql.connector import Error
from datetime import datetime, timedelta
import random

class DatabaseManager:
    def __init__(self, host, user, password, database):
        self.host = host
        self.user = user
        self.password = password
        self.database = database
        self.connection = None

    def connect(self):
        try:
            if self.connection is None:
                self.connection = mysql.connector.connect(
                    host=self.host,
                    user=self.user,
                    password=self.password,
                    database=self.database
                )
                print("Connection to MYSQL DB successful")
        except Error as e:
            print(f"Error: '{e}' occurred")

    def disconnect(self):
        if self.connection is not None and self.connection.is_connected():
            self.connection.close()
            self.connection = None
            print("Disconnected")

    def _executeQuery(self, query, params=None, fetch=False, many=False):
        if self.connection is None or not self.connection.is_connected():
            print("Not connected to the database")
            return None

        try:
            cursor = self.connection.cursor()
            if many:
                cursor.executemany(query, params)
            else:
                cursor.execute(query, params)
            if fetch:
                result = cursor.fetchall()
                cursor.close()
                return result
            self.connection.commit()
            cursor.close()
        except Error as e:
            print(f"Error: '{e}' occurred")
            return None

    def insertSalesData(self, data):
        query = """
        INSERT INTO sales (order_id, order_datetime, menu_id, topping_id, quantity, price, gender, age)
        VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
        """
        self._executeQuery(query, data, fetch=False, many=True)
        print("Data inserted successfully")

    def updateStock(self, menuData, toppingData):
        self.updateMenuStock(menuData)
        self.updateToppingStock(toppingData)

    def updateMenuStock(self, data):
        query = "UPDATE menu SET stock = %s WHERE name = %s"
        for itemName, itemStock in data:
            # print(f"Updating menu item: {itemName} with stock: {itemStock}")
            self._executeQuery(query, (itemStock, itemName), fetch=False)
        print("Menu stock updated successfully")

    def updateToppingStock(self, data):
        query = "UPDATE topping SET stock = %s WHERE name = %s"
        for itemName, itemStock in data:
            # print(f"Updating topping item: {itemName} with stock: {itemStock}")
            self._executeQuery(query, (itemStock, itemName), fetch=False)
        print("Topping stock updated successfully")

    def getDailySales(self, date):
        query = """
        SELECT s.order_id, TIME(s.order_datetime) AS order_time, m.name, t.name, s.quantity, s.price
        FROM sales s
        JOIN menu m ON s.menu_id = m.id
        LEFT JOIN topping t ON s.topping_id = t.id
        WHERE DATE(s.order_datetime) = %s
        ORDER BY s.order_id
        """
        return self._executeQuery(query, (date,), fetch=True)

    def getDailyTotalSales(self, year, month):
        query = """
        SELECT DATE(order_datetime) as date, SUM(price * quantity) as total_sales
        FROM sales
        WHERE Year(order_datetime) = %s and MONTH(order_datetime) = %s
        GROUP BY date
        ORDER BY date
        """
        return self._executeQuery(query, (year, month), fetch=True)

    def getMonthTotalSales(self, year, month):
        query = """
        SELECT SUM(price * quantity) as total_sales
        FROM sales
        WHERE Year(order_datetime) = %s AND MONTH(order_datetime) = %s
        """
        return self._executeQuery(query, (year, month), fetch=True)

    def getStocks(self):
        menuQuery = "SELECT name, stock FROM menu"
        toppingQuery = "SELECT name, stock FROM topping"

        menuResult = self._executeQuery(menuQuery, fetch=True)
        toppingResult = self._executeQuery(toppingQuery, fetch=True)

        return menuResult, toppingResult

    def getMenuSales(self, date):
        query = """
        SELECT m.name, SUM(s.quantity) as total_quantity
        FROM sales as s
        JOIN menu m ON s.menu_id = m.id
        WHERE DATE(order_datetime) = %s
        GROUP BY m.name
        """
        return self._executeQuery(query, (date,), fetch=True)
    
    def getHourlySales(self, date):
        query = """
        SELECT DATE_FORMAT(order_datetime, '%H') AS hour, SUM(quantity)
        FROM sales
        WHERE DATE(order_datetime) = %s
        GROUP BY hour
        ORDER BY hour;
        """
        return self._executeQuery(query, (date,), fetch=True)
    
    def getRestQuantity(self):
        menuQuery = "SELECT name, stock FROM menu"
        toppingQuery = "SELECT name, stock FROM topping"

        menuResult = self._executeQuery(menuQuery, fetch=True)
        toppingResult = self._executeQuery(toppingQuery, fetch=True)

        return menuResult, toppingResult
    
    def getMenuToppingPreferences(self, year, month, day):
        query = """
        SELECT 
            m.name AS menu_name,
            t.name AS topping_name,
            COUNT(*) AS count
        FROM 
            sales s
        JOIN 
            menu m ON s.menu_id = m.id
        JOIN 
            topping t ON s.topping_id = t.id
        WHERE 
            DATE(s.order_datetime) = %s
            AND m.name != '아포가토'  -- '아포가토'를 제외
        GROUP BY 
            m.name, t.name
        ORDER BY 
            count DESC;
        """
        
        # 날짜 포맷: 'YYYY-MM-DD'
        date_str = f"{year:04d}-{month:02d}-{day:02d}"
        result = self._executeQuery(query, (date_str,), fetch=True)
        
        menu_names = []
        topping_names = []
        counts = []
        
        # 결과를 튜플로 처리
        for row in result:
            # 결과 확인
            # print(f"Row: {row}")
            
            try:
                # 튜플의 인덱스에 맞춰 데이터 접근
                menu_name = row[0]
                topping_name = row[1]
                count = int(row[2])  # 판매 개수를 int로 변환
                
                menu_names.append(menu_name)
                topping_names.append(topping_name)
                counts.append(count)
            
            except ValueError as e:
                print(f"Error processing row: {row}. Exception: {e}")
        
        # print(f"Menu Names: {menu_names}")
        # print(f"Topping Names: {topping_names}")
        # print(f"Counts: {counts}")

        return menu_names, topping_names, counts


    def getSalesByAgeGroup(self, year, month, day):
        date_str = f"{year}-{month:02d}-{day:02d}"

        query = """
        SELECT 
            CASE
                WHEN age BETWEEN 0 AND 9 THEN '0-9'
                WHEN age BETWEEN 10 AND 19 THEN '10-19'
                WHEN age BETWEEN 20 AND 29 THEN '20-29'
                WHEN age BETWEEN 30 AND 39 THEN '30-39'
                WHEN age BETWEEN 40 AND 49 THEN '40-49'
                WHEN age BETWEEN 50 AND 59 THEN '50-59'
                ELSE '60+'
            END AS age_group,
            SUM(quantity) AS total_sales
        FROM 
            sales
        WHERE 
            DATE(order_datetime) = %s
        GROUP BY 
            age_group
        ORDER BY 
            age_group;
        """
        
        # Execute the query and return results
        result = self._executeQuery(query, (date_str,), fetch=True)
        return result
    
##################################################################################
    #덤프 데이터 생성
    def insertDummyData(self, numRecords, startDate, endDate):
        genders = ['M', 'F']
        menu_ids = [1, 2, 3, 4]  # menu 테이블의 id 값
        topping_ids = [1, 2, 3]  # topping 테이블의 id 값 (임시로 3개의 토핑이 있다고 가정)
        menu_prices = {1: 3000, 2: 3000, 3: 3000, 4: 4000}

        total_days = (endDate - startDate).days + 1
        records_per_day = [random.randint(1, numRecords // total_days * 2) for _ in range(total_days)]
        records_per_day[-1] = numRecords - sum(records_per_day[:-1])  # 나머지 레코드를 마지막 날에 할당

        record_count = 0
        data = []

        for day in range(total_days):
            daily_records = []
            currentDate = startDate + timedelta(days=day)

            for _ in range(records_per_day[day]):
                if record_count >= numRecords or currentDate > endDate:
                    break

                # 시간대별로 랜덤하게 분포시키기 위해 시간을 랜덤하게 생성
                random_hour = random.randint(0, 23)
                random_minute = random.randint(0, 59)
                random_second = random.randint(0, 59)
                order_datetime = currentDate + timedelta(hours=random_hour, minutes=random_minute, seconds=random_second)
                
                menu_id = random.choice(menu_ids)
                topping_id = random.choice(topping_ids)
                quantity = 1
                price = menu_prices[menu_id]
                gender = random.choice(genders)
                age = random.randint(10, 60)

                daily_records.append((order_datetime, menu_id, topping_id, quantity, price, gender, age))

                record_count += 1

            # daily_records를 시간 순으로 정렬
            daily_records.sort(key=lambda x: x[0])
            data.extend(daily_records)

        # 전체 데이터를 시간 순으로 정렬
        data.sort(key=lambda x: x[0])

        # 정렬된 데이터에 오더 넘버 부여
        final_data = []
        previous_date = None
        order_id = 1

        for record in data:
            order_datetime = record[0]
            if previous_date is None or previous_date.date() != order_datetime.date():
                order_id = 1  # 날짜가 변하면 order_id를 1로 초기화
                previous_date = order_datetime

            final_data.append((order_id, *record))
            order_id += 1

        self.insertSalesData(final_data)
        print(f"{record_count}개의 덤프 데이터가 삽입되었습니다.")

    def truncateTable(self):
        query = "TRUNCATE TABLE sales"
        self._executeQuery(query, fetch=False)
        print("sales 테이블이 초기화되었습니다.")

