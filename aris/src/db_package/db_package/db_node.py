from datetime import datetime
from rclpy.node import Node
from interface_package.msg import StockInfo, StocksArray, OrderInfo, Item
from interface_package.srv import DailyTotalSales, MonthTotalSales, Stocks, ModifyStocks, DailySales, MenuDailySales, HourlySales, OrderRecord, RestQuantity, MenuTopping, Age

from db_package.db import DatabaseManager

class DataBaseNode(Node):
    def __init__(self):
        super().__init__('database_node')
        self.initDataBase()
        self.initServices()

    def initDataBase(self):
        self.dbManager = DatabaseManager(
                host="localhost",
                user="root",
                password="amr231218!",
                database="ArisTeam5"
            )
        self.dbManager.connect()

    def initServices(self):
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
        
    def initClients(self):
        self.updateStocksService = self.create_client(ModifyStocks, "updateStocks")


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
        
        try:
            results = self.dbManager.getMonthTotalSales(year, month)
            if results and results[0] and results[0][0] is not None:
                response.total_sales = int(results[0][0])
            else:
                response.total_sales = 0
                self.get_logger().warn(f"No sales data found for year: {year}, month: {month}")
        except Exception as e:
            self.get_logger().error(f"Error processing monthly total sales: {e}")
            response.total_sales = 0
        
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

        self.requestUpdateStocks(menuStockData, toppingStockData)

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

    def requestUpdateStocks(self, menuData, toppingData):
        request = ModifyStocks.Request()
        request.stocks.menu = menuData
        request.stocks.topping = toppingData

        self.get_logger().info("Request stock updates")
        
        future = self.updateStocksClient.call_async(request)
        future.add_done_callback(self.modifyStocksCallback)

    def updateStocksCallback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Stocks successfully updated")
            else:
                self.get_logger().info("Failed to update stocks")
        except Exception as e:
            self.get_logger().error(f"Update Stocks service call failed: {e}")
