from datetime import datetime
from rclpy.node import Node
from interface_package.msg import StockInfo, StocksArray, OrderInfo, Item
from interface_package.srv import DailyTotalSales, MonthTotalSales, Stocks, ModifyStocks, DailySales, MenuDailySales, HourlySales, OrderRecord, RestQuantity, MenuTopping, Age
from db_package.db_manager import DatabaseManager
from db_package.config_loader import DatabaseConfig

class DataBaseNode(Node):
    """
        데이터베이스 노드
    """

    def __init__(self):
        super().__init__('database_node')
        self.db_config = DatabaseConfig
        self.initDataBase()
        self.initServices()

    def initDataBase(self):
        """
        데이터베이스 접속
        """
        try:
            if self.db_config and all(key in self.db_config for key in ['host', 'user', 'password', 'name']):
                self.dbManager = DatabaseManager(
                    host=self.db_config['host'],
                    user=self.db_config['user'],
                    password=self.db_config['password'],
                    database=self.db_config['name']
                )
                self.dbManager.connect()
            else:
                raise ValueError("Invalid database configuration")
        except Exception as e:
            print(f"An error occurred: {e}")

    def initServices(self):
        """
        서비스 정의
        """
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
        """
        클라이언트 정의
        """
        self.updateStocksService = self.create_client(ModifyStocks, "updateStocks")

    def dailyTotalSalesCallback(self, request, response):
        """
        일별 총 판매량 서비스 콜백
        Args:
        request: 서비스 요청 객체
            - year (str): 조회할 연도
            - month (str): 조회할 월 (1부터 12까지의 값)
        response: 서비스 응답 객체
            - data (list of str): 일별 총 판매량 데이터를 담고 있는 문자열 리스트
            - 각 문자열은 "날짜,총판매량" 형식

        Returns:
        response: 요청에 대한 응답 객체로, `data` 속성에 조회된 판매량 데이터
        """
        year = request.year
        month = request.month
        self.get_logger().info(f"Get request from dailyTotalSalesClient year: {year}, month: {month}")

        try:
            results = self.dbManager.getDailyTotalSales(year, month)
            if results is None:
                self.get_logger().warning(f"No results found for year {year} and month {month}")
                response.data = []
            else:
                response.data = [f"{record[0]},{record[1]}" for record in results]
        except Exception as e:
            self.get_logger().error(f"An error occurred while processing the request: {e}")
            response.data = []

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
        date_str = request.date 
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

        try:
            order_datetime = datetime.strptime(date_str, "%Y%m%d %H:%M")
        except ValueError as e:
            response.success = False
            return response

        data = [(order_number, order_datetime, menu_id, topping_id, quantity, price, gender, age)]

        self.dbManager.insertSalesData(data)
        
        
        curMenuStocks, curToppingStocks = self.dbManager.getRestQuantity()

        menu_stock_dict = {name: stock for name, stock in curMenuStocks}
        topping_stock_dict = {name: stock for name, stock in curToppingStocks}

        if menu in menu_stock_dict:
            current_menu_stock = menu_stock_dict[menu]
            new_menu_stock = current_menu_stock - quantity

            menuStockData = [(menu, new_menu_stock)]
            self.dbManager.updateMenuStock(menuStockData)

        if topping in topping_stock_dict:
            current_topping_stock = topping_stock_dict[topping]
            new_topping_stock = current_topping_stock - quantity
        
        toppingStockData = [(topping, new_topping_stock)]
        self.dbManager.updateToppingStock(toppingStockData)

        self.requestUpdateStocks(menuStockData, toppingStockData)

        return response
    
    def menuToppingCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day

        menu_names, topping_names, counts = self.dbManager.getMenuToppingPreferences(year, month, day)

        if not (len(menu_names) == len(topping_names) == len(counts)):
            raise ValueError("Mismatch in lengths of menu_names, topping_names, and counts lists")
        
        self.get_logger().info(f"Fetched Menu Names: {menu_names}")
        self.get_logger().info(f"Fetched Topping Names: {topping_names}")
        self.get_logger().info(f"Fetched Counts: {counts}")
        
        response.menu_names = menu_names
        response.topping_names = topping_names
        response.counts = counts
        
        return response

    def ageCallback(self, request, response):
        year = request.year
        month = request.month
        day = request.day
        
        age_data = self.dbManager.getSalesByAgeGroup(year, month, day)
        
        age_groups = []
        age_group_sales = []
        
        for data in age_data:
            age_groups.append(data[0])
            
            try:
                sales = int(data[1])
            except ValueError:
                raise ValueError(f"Expected int type for sales count but got {type(data[1])} with value {data[1]}")
            
            if not (-2147483648 <= sales <= 2147483647):
                raise ValueError(f"Sales count {sales} is out of range for int32")
            
            age_group_sales.append(sales)
        
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
