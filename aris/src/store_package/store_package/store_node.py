import rclpy as rp
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from PyQt5.QtCore import QObject, pyqtSignal
from interface_package.msg import StockInfo, StocksArray, OrderInfo, RobotStatusInfo
from interface_package.srv import DailyTotalSales, MonthTotalSales, Stocks, ModifyStocks, DailySales, MenuDailySales, HourlySales, MenuTopping, Age


class StoreNode(Node, QObject):
    dailyTotalSales = pyqtSignal(object)
    monthTotalSales = pyqtSignal(object)
    stocks = pyqtSignal(object)
    dailySales = pyqtSignal(object)
    menuDailySales = pyqtSignal(object)
    hourlySales = pyqtSignal(object)
    menuToppingSales = pyqtSignal(list, list, list)
    ageSales = pyqtSignal(list, list)
    robotStatus = pyqtSignal(list, list)
    allServicesAvailable = pyqtSignal()

    def __init__(self):
        super().__init__("store_node")
        QObject.__init__(self)
        self.initNode()
            
    def initNode(self):
        self.get_logger().info("Store node started")
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.initClients()
        self.initServices()
        self.initSub()
        self.waitServices()

    def initClients(self):
        self.dailyTotalSalesClient = self.create_client(DailyTotalSales, 'dailyTotalSales')
        self.monthTotalSalesClient = self.create_client(MonthTotalSales, 'monthTotalSales')
        self.stocksClient = self.create_client(Stocks, 'stocks')
        self.modifyStocksClient = self.create_client(ModifyStocks, 'modifyStocks')
        self.dailySalesClient = self.create_client(DailySales, "dailySales")
        self.menuDailySalesClient = self.create_client(MenuDailySales, "menuDailySales")
        self.hourlySalesClient = self.create_client(HourlySales, 'hourlySales')
        self.menuToppingClient = self.create_client(MenuTopping, 'MenuTopping')
        self.ageClient = self.create_client(Age, 'Age')

    def initServices(self):
        self.updateStocksClient = self.create_service(ModifyStocks, 'updateStocks', self.updateStocksCallback)

    def initSub(self):
        self.jointTempSub = self.create_subscription(RobotStatusInfo, 'RobotStatusInfo', self.robotStatusInfocallback, self.qos_profile)

    def waitServices(self):
        self.timer = self.create_timer(1.0, self.checkServices)

    def checkServices(self):
        all_services_available = True

        if not self.dailyTotalSalesClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Daily total sales service not available, waiting again...')
            all_services_available = False

        if not self.monthTotalSalesClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Month total sales service not available, waiting again...')
            all_services_available = False

        if not self.stocksClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Stocks service not available, waiting again...')
            all_services_available = False

        if not self.modifyStocksClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Modify Stocks service not available, waiting again...')
            all_services_available = False

        if not self.dailySalesClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Daily sales service not available, waiting again...')
            all_services_available = False

        if not self.menuDailySalesClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Menu daily sales service not available, waiting again...')
            all_services_available = False

        if not self.hourlySalesClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Hourly sales service not available, waiting again...')
            all_services_available = False
            
        if not self.menuToppingClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('MenuTopping service not available, waiting again...')
            all_services_available = False

        if not self.ageClient.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('Age service not available, waiting again...')
            all_services_available = False

        if all_services_available:
            self.get_logger().info('All services are now available.')
            self.timer.cancel()
            self.allServicesAvailable.emit()

    def requestDailyTotalSales(self, year, month):
        request = DailyTotalSales.Request()
        request.year = year
        request.month = month

        self.get_logger().info(f"request to DailyTotalSales service, year : {year}, month : {month}")

        future = self.dailyTotalSalesClient.call_async(request)
        future.add_done_callback(self.dailyTotalSalesCallback)

    def dailyTotalSalesCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from dailyTotalSalesService")
            self.dailyTotalSales.emit(response.data)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def requestMonthTotalSales(self, year, month):
        request = MonthTotalSales.Request()
        request.year = year
        request.month = month

        future = self.monthTotalSalesClient.call_async(request)
        future.add_done_callback(self.monthTotalSalesCallback)
    
    def monthTotalSalesCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from monthTotalSalesService")
            self.monthTotalSales.emit(response.total_sales)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def requestStocks(self):
        request = Stocks.Request()

        future = self.stocksClient.call_async(request)
        future.add_done_callback(self.stocksCallback)

    def stocksCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from stockService")

            self.get_logger().info(f"Menu Items:")
            for item in response.stocks.menu:
                self.get_logger().info(f"Name: {item.name}, Stock: {item.stock}")
            
            self.get_logger().info(f"Toppings:")
            for item in response.stocks.topping:
                self.get_logger().info(f"Name: {item.name}, Stock: {item.stock}")
            
            self.stocks.emit(response.stocks)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def requestModifyStocks(self, menuData, toppingData):
        request = ModifyStocks.Request()
        request.stocks.menu = menuData
        request.stocks.topping = toppingData

        self.get_logger().info("Request stock modification")
        
        future = self.modifyStocksClient.call_async(request)
        future.add_done_callback(self.modifyStocksCallback)

    def modifyStocksCallback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Stocks successfully modified")
            else:
                self.get_logger().info("Failed to modify stocks")
        except Exception as e:
            self.get_logger().error(f"Modify Stocks service call failed: {e}")

    def requestDailySales(self, year, month, day):
        request = DailySales.Request()
        request.year = year
        request.month = month
        request.day = day

        future = self.dailySalesClient.call_async(request)
        future.add_done_callback(self.dailySalesCallback)
    
    def dailySalesCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from dailySalesService")
            self.dailySales.emit(response.data)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def requestMenuDailySales(self, year, month, day):
        request = MenuDailySales.Request()
        request.year = year
        request.month = month
        request.day = day

        self.get_logger().info(f"Request to MenuDailySales service - year: {year}, month: {month}, day: {day}")

        future = self.menuDailySalesClient.call_async(request)
        future.add_done_callback(self.menuDailySalesCallback)

    def menuDailySalesCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from MenuDailySales service")
            self.menuDailySales.emit(response.items)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def requestHourlySales(self, year, month, day):
        request = HourlySales.Request()
        request.year = year
        request.month = month
        request.day = day

        self.get_logger().info(f"Request to HourlySales service - year: {year}, month: {month}, day: {day}")

        future = self.hourlySalesClient.call_async(request)
        future.add_done_callback(self.hourlySalesCallback)

    def hourlySalesCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from hourlySales service")
            self.hourlySales.emit(response.items)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def robotStatusInfocallback(self, msg):
        joints = [f"{msg.j1:.2f}", f"{msg.j2:.2f}", f"{msg.j3:.2f}", f"{msg.j4:.2f}", f"{msg.j5:.2f}", f"{msg.j6:.2f}"]
        temperatures = [f"{msg.temperature1:.2f}", f"{msg.temperature2:.2f}", f"{msg.temperature3:.2f}", f"{msg.temperature4:.2f}", f"{msg.temperature5:.2f}", f"{msg.temperature6:.2f}"]
        self.robotStatus.emit(joints, temperatures)
        self.get_logger().info(f'\n Joints: {joints}\n Temperatures: {temperatures}')

    def requestMenuToppingSales(self, year, month, day):
        request = MenuTopping.Request()
        request.year = year
        request.month = month
        request.day = day
        self.get_logger().info(f"Request to MenuToppingSales service - year: {year}, month: {month}, day: {day}")

        future = self.menuToppingClient.call_async(request)
        future.add_done_callback(self.menuToppingSalesCallback)
        
    def menuToppingSalesCallback(self, future):
        try:
            response = future.result()
            
            menu_names = response.menu_names
            topping_names = response.topping_names
            counts = list(response.counts)

            self.get_logger().info(f"Menu Names: {menu_names}")
            self.get_logger().info(f"Topping Names: {topping_names}")
            self.get_logger().info(f"Counts: {counts}")

            self.menuToppingSales.emit(menu_names, topping_names, counts)

        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def requestAgeSales(self, year, month, day):
        request = Age.Request()
        request.year = year
        request.month = month
        request.day = day
        self.get_logger().info(f"Request to AgeSales service - year: {year}, month: {month}, day: {day}")

        future = self.ageClient.call_async(request)
        future.add_done_callback(self.ageSalesCallback)

    def ageSalesCallback(self, future):
        try:
            response = future.result()
            age_groups = response.age_groups
            age_group_sales = list(response.age_group_sales)
            self.get_logger().info(f"Age Groups: {age_groups}")
            self.get_logger().info(f"Age Group Sales: {age_group_sales}")

            self.ageSales.emit(age_groups, age_group_sales)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def updateStocksCallback(self, request, response):
        self.get_logger().info(f"Get request from updateStocksClient")
        try:
            self.stocks.emit(request.stocks)
            response.success = True
            self.get_logger().info("Stocks successfully update in UI")
        except Exception as e:
            response.success = False
            self.get_logger().error(f"Failed to update stocks in UI: {e}")

        return response