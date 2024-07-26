import mysql.connector
from mysql.connector import Error
from datetime import datetime, timedelta
import random
import rclpy as rp
from rclpy.node import Node
from interface_package.msg import StockInfo, StocksArray, OrderInfo, Item
from interface_package.srv import DailyTotalSales, MonthTotalSales, Stocks, ModifyStocks, DailySales, MenuDailySales, HourlySales, OrderRecord, RestQuantity, MenuTopping, Age

class DataBaseNode(Node):
    def __init__(self):
        super().__init__('database_node')
        self.initDataBase()
        self.initService()

    def initDataBase(self):
        self.dbManager = DatabaseManager(
                host="localhost",
                user="root",
                password="amr231218!",
                database="ArisTeam5"
            )
        self.dbManager.connect()

    def initService(self):
        self.dailyTotalSalesService = self.create_service(DailyTotalSales, "dailyTotalSales", self.dailyTotalSalesCallback)
        self.monthTotalSalesService = self.create_service(MonthTotalSales, "monthTotalSales", self.monthTotalSalesCallback)
        self.stocksService = self.create_service(Stocks, "stocks", self.stocksCallback)
        self.modifyStocksService = self.create_service(ModifyStocks, "modifyStocks", self.modifyStocksCallback)
        self.dailySalesService = self.create_service(DailySales, "dailySales", self.dailySalesCallback)
        self.MenuDailySalesService = self.create_service(MenuDailySales, "menuDailySales", self.menuDailySalesCallback)
        self.hourlySalesService = self.create_service(HourlySales, 'hourlySales', self.hourlySalesCallback)
        self.orderRecordService = self.create_service(OrderRecord, 'OrderRecord', self.orderRecordCallback)
        self.restQuantityService = self.create_service(RestQuantity, 'RestQuantity', self.restQuantityCallback)
        self.menuToppingService = self.create_service(MenuTopping, 'MenuTopping', self.menuToppingCallback)
        self.ageService = self.create_service(Age, 'Age', self.ageCallback)
        
    def dailyTotalSalesCallback(self, request, response):
        year = request.year
        month = request.month
        self.get_logger().info(f"Get request from dailyTotalSalesClient year : {year}, month : {month}")
        results = self.dbManager.getDailyTotalSales(year, month)
        response.data = [f"{record[0]},{record[1]}" for record in results]

        return response
    
    def monthTotalSalesCallback(self, request, response):
        year = request.year
        month = request.month
        self.get_logger().info(f"Get request from monthTotalSalesClient year : {year}, month : {month}")
        results = self.dbManager.getMonthTotalSales(year, month)
        response.total_sales = int(results[0][0])

        return response

    def stocksCallback(self, request, response):
        self.get_logger().info(f"Get request from stocksClient")
        menu_result, topping_result = self.dbManager.getStocks()
        if menu_result is not None and topping_result is not None:
            menu_items = [StockInfo(name=row[0], stock=row[1]) for row in menu_result]
            toppings = [StockInfo(name=row[0], stock=row[1]) for row in topping_result]
            response.stocks = StocksArray(menu=menu_items, topping=toppings)
        else:
            response.stocks = StocksArray(menu=[], topping=[])

        self.get_logger().info(f"Sending response with {len(response.stocks.menu)} menu items and {len(response.stocks.topping)} toppings")
        return response

    def modifyStocksCallback(self, request, response):
        self.get_logger().info(f"Get request from modifyStocksClient")
        
        menuData = [(item.name, item.stock) for item in request.stocks.menu]
        toppingData = [(item.name, item.stock) for item in request.stocks.topping]

        try:
            self.dbManager.updateStock(menuData, toppingData)
            response.success = True
            self.get_logger().info("Stocks successfully modified in database")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to modify stocks in database: {e}")

        return response

    def dailySalesCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day
        
        self.get_logger().info(f"Get request from dailySalesClient - year: {year}, month: {month}, day: {day}")
        date_str = f"{year:04d}-{month:02d}-{day:02d}"
        date_obj = datetime.strptime(date_str, "%Y-%m-%d").date()
        results = self.dbManager.getDailySales(date_obj)

        # response.data = [
        #     OrderInfo(
        #         order_id = row[0],
        #         order_time = row[1],
        #         menu = row[2],
        #         topping = row[3],
        #         quantity = row[4],
        #         price = row[5],
        #     ) for row in results
        #     ] 
    
        orderInfoList = []
        for row in results:
            orderInfo = OrderInfo(
                order_id = row[0],
                order_time =  str(row[1]),
                menu = row[2],
                topping = row[3],
                quantity = row[4],
                price = row[5],
            ) 
            orderInfoList.append(orderInfo)
        
        response.data = orderInfoList

        return response

    def menuDailySalesCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day
        date = f"{year:04d}-{month:02d}-{day:02d}"

        self.get_logger().info(f"Get request from menuDailySalesClient - year: {year}, month: {month}, day: {day}")

        results = self.dbManager.getMenuSales(date)

        response.items = [
            Item(name=row[0], quantity=int(row[1])) for row in results
        ]

        return response
    
    def hourlySalesCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day
        date = f"{year:04d}-{month:02d}-{day:02d}"

        self.get_logger().info(f"Get request from menuHourlySalesClient - year: {year}, month: {month}, day: {day}")

        results = self.dbManager.getHourlySales(date)

        if results is None:
            self.get_logger().error(f"Failed to retrieve data for date: {date}")
            response.items = []
        else:
            response.items = [
                Item(name=row[0], quantity=int(row[1])) for row in results
            ]
        print(response.items)

        return response
    
    def restQuantityCallback(self, request, response):
        menuresults, toppingresults = self.dbManager.getRestQuantity()
        for menu in menuresults:
            if menu[0] == "딸기":
                response.strawberry = menu[1]
            elif menu[0] == "바나나":
                response.banana = menu[1]
            elif menu[0] == "초코":
                response.chocolate = menu[1]
            elif menu[0] == "아포가토":
                response.affogato = menu[1]

        for topping in toppingresults:
            if topping[0] == "토핑A":
                response.topping_a = topping[1]
            elif topping[0] == "토핑B":
                response.topping_b = topping[1]
            elif topping[0] == "토핑C":
                response.topping_c = topping[1]

        return response

    def orderRecordCallback(self, request, response):
        self.get_logger().info(f"date : {type(request.date)}")
        order_number = request.order_number
        date_str = request.date  # 문자열 형식의 날짜
        menu = request.menu
        topping = request.topping
        price = request.price
        quantity = request.quantity
        gender = request.gender
        age = request.age

        if menu == "berry":
            menu = "딸기"
            menu_id = 1
        elif menu == "banana":
            menu = "바나나"
            menu_id = 2
        elif menu == "choco":
            menu = "초코"
            menu_id = 3
        elif menu == "affogato" or menu == "아포가토":
            menu = "이포가토"
            menu_id = 4

        if topping == "topping A":
            topping = "토핑A"
            topping_id = 1
        elif topping == "topping B":
            topping = "토핑B"
            topping_id = 2
        elif topping == "topping C":
            topping = "토핑C"
            topping_id = 3

        if gender == "male":
            gender = "M"
        elif gender == "female":
            gender = "F"

        # 문자열을 datetime 객체로 변환 (형식에 맞게 수정 필요)
        try:
            order_datetime = datetime.strptime(date_str, "%Y%m%d %H:%M")
        except ValueError as e:
            response.success = False
            return response

        data = [(order_number, order_datetime, menu_id, topping_id, quantity, price, gender, age)]

        # 데이터 삽입
        self.dbManager.insertSalesData(data)
        
        
        curMenuStocks, curToppingStocks = self.dbManager.getRestQuantity()

        menu_stock_dict = {name: stock for name, stock in curMenuStocks}
        topping_stock_dict = {name: stock for name, stock in curToppingStocks}

        # 현재 재고 확인 및 업데이트
        if menu in menu_stock_dict:
            current_menu_stock = menu_stock_dict[menu]
            new_menu_stock = current_menu_stock - quantity
            # 메뉴 재고 업데이트
            menuStockData = [(menu, new_menu_stock)]
            self.dbManager.updateMenuStock(menuStockData)

        if topping in topping_stock_dict:
            current_topping_stock = topping_stock_dict[topping]
            new_topping_stock = current_topping_stock - quantity
            # 토핑 재고 업데이트
        
        toppingStockData = [(topping, new_topping_stock)]
        self.dbManager.updateToppingStock(toppingStockData)
        return response
    
    def menuToppingCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day
        
        # 메뉴와 토핑의 판매 데이터 가져오기
        menu_names, topping_names, counts = self.dbManager.getMenuToppingPreferences(year, month, day)
        
        # 데이터 검증 및 응답 설정
        if not (len(menu_names) == len(topping_names) == len(counts)):
            raise ValueError("Mismatch in lengths of menu_names, topping_names, and counts lists")
        
        # 로깅
        self.get_logger().info(f"Fetched Menu Names: {menu_names}")
        self.get_logger().info(f"Fetched Topping Names: {topping_names}")
        self.get_logger().info(f"Fetched Counts: {counts}")
        
        # 응답에 데이터 설정
        response.menu_names = menu_names
        response.topping_names = topping_names
        response.counts = counts
        
        return response

    def ageCallback(self, request, response):
        # 요청 데이터에서 날짜 추출
        year = request.year
        month = request.month
        day = request.day
        
        # 나이대별 판매 데이터 가져오기
        age_data = self.dbManager.getSalesByAgeGroup(year, month, day)
        
        # 데이터 검증 및 응답 설정
        age_groups = []
        age_group_sales = []
        
        for data in age_data:
            # 나이대 추가
            age_groups.append(data[0])
            
            # 판매량 검증 및 변환
            try:
                sales = int(data[1])  # 판매량을 명시적으로 int로 변환
            except ValueError:
                raise ValueError(f"Expected int type for sales count but got {type(data[1])} with value {data[1]}")
            
            # 판매량이 int32 범위 내에 있는지 확인
            if not (-2147483648 <= sales <= 2147483647):
                raise ValueError(f"Sales count {sales} is out of range for int32")
            
            # 판매량 추가
            age_group_sales.append(sales)
        
        # 응답 필드 설정
        response.age_groups = age_groups
        response.age_group_sales = age_group_sales
        
        return response


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

def main():
    rp.init(args=None)
    dbNode = DataBaseNode()
    rp.spin(dbNode)
    dbNode.destroy_node()
    rp.shutdown()


if __name__ == "__main__":
    # main()
    
    # # 테스트용
    dbManager = DatabaseManager(
        host="localhost",
        user="root",
        password="amr231218!",
        database="ArisTeam5"
    )

    try:
        dbManager.connect()

        # 덤프 데이터 넣기
        dbManager.truncateTable()
        startDate = datetime.strptime('2024-07-04 00:00:00', '%Y-%m-%d %H:%M:%S')
        endDate = datetime.strptime('2024-07-17 23:59:59', '%Y-%m-%d %H:%M:%S')
        dbManager.insertDummyData(1000, startDate, endDate)

    finally:
        dbManager.disconnect()
