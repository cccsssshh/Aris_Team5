import sys
import os
import glob
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDateTime, QDate, Qt, QRect, QObject, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QColor, QFont, QPainter, QPixmap, QImage, QPen
import netifaces
import threading


import rclpy as rp
from rclpy.node import Node
from interface_package.msg import StockInfo, StocksArray, OrderInfo
from interface_package.srv import DailyTotalSales, MonthTotalSales, Stocks, ModifyStocks, DailySales, MenuDailySales

#학원
uiPath = "/home/addinedu/amr_ws/aris_team5/Aris_Team5/src/store_package/ui"
#집
# uiPath = "/home/sungho/amr_ws/git_ws/Aris_Team5/src/store_package/ui"

loginUi = glob.glob(os.path.join(uiPath, "login.ui"))[0]
mainUi = glob.glob(os.path.join(uiPath, "store.ui"))[0]
robotUi = glob.glob(os.path.join(uiPath, "robot_manage.ui"))[0]
dailySalesUi = glob.glob(os.path.join(uiPath, "daily_sales.ui"))[0]

loginClass = uic.loadUiType(loginUi)[0]
mainClass = uic.loadUiType(mainUi)[0]
robotClass = uic.loadUiType(robotUi)[0]
dailySalesClass = uic.loadUiType(dailySalesUi)[0]

class StoreNode(Node, QObject):
    dailyTotalSales = pyqtSignal(object)
    monthTotalSales = pyqtSignal(object)
    stocks = pyqtSignal(object)
    dailySales = pyqtSignal(object)
    menuDailySales = pyqtSignal(object)
    jointTemperature = pyqtSignal(object)

    def __init__(self):
        super().__init__("store_node")
        QObject.__init__(self)
        self.initNode()
            
    def initNode(self):
        self.get_logger().info("Store node started")
        # time.sleep(0.5)  # Wait for 2 seconds to ensure all services are up
        self.initClients()
        self. waitService()

    def initClients(self):
        self.dailyTotalSalesClient = self.create_client(DailyTotalSales, 'dailyTotalSales')
        self.monthTotalSalesClient = self.create_client(MonthTotalSales, 'monthTotalSales')
        self.stocksClient = self.create_client(Stocks, 'stocks')
        self.modifyStocksClient = self.create_client(ModifyStocks, 'modifyStocks')
        self.dailySalesClient = self.create_client(DailySales, "dailySales")
        self.menuDailySalesClient = self.create_client(MenuDailySales, "menuDailySales")

    # def initSub(self):
    #     self.jointTempSub = self.create_subscription(JointTemperature, "jointTemperature", callback)

    def waitService(self):
        while not self.dailyTotalSalesClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Daily total sales service not available, waiting again...')
        while not self.monthTotalSalesClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Month total sales service not available, waiting again...')
        while not self.stocksClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stocks service not available, waiting again...')
        while not self.modifyStocksClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Modify Stocks service not available, waiting again...')
        while not self.dailySalesClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Daily sales service not available, waiting again...')
        while not self.menuDailySalesClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Menu daily sales service not available, waiting again...')


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
            
            # 메뉴 아이템 출력
            self.get_logger().info(f"Menu Items:")
            for item in response.stocks.menu:
                self.get_logger().info(f"Name: {item.name}, Stock: {item.stock}")
            
            # 토핑 아이템 출력
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
        future.add_done_callback(self.requestMenuDailySalesCallback)

    def requestMenuDailySalesCallback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Received response from MenuDailySales service")
            self.menuDailySales.emit(response.items)
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
    
    def jointTemperatureCallback(self):
        #메세지 받으면 emit하기
        pass


class Calendar(QCalendarWidget):
    dateClicked = pyqtSignal(QDate)

    def __init__(self, parent=None):
        super(Calendar, self).__init__(parent)
        self.salesRecords = {}
        self.initWindow()

    def initWindow(self):
        self.clicked.connect(self.selectDate)

    def setDailyTotalSales(self, salesRecords):
        self.salesRecords = salesRecords
        self.updateCells()

    def paintCell(self, painter, rect, date):
        super(Calendar, self).paintCell(painter, rect, date)
        
        if date in self.salesRecords:
            painter.save()
            painter.setPen(QColor(0, 100, 0))
            painter.setFont(QFont("Arial", 8, QFont.Bold))
            textRect = QRect(rect.left(), rect.top() + 25, rect.width(), rect.height() - 20)
            painter.drawText(textRect, Qt.AlignCenter, self.salesRecords[date])
            painter.restore()
    
    def selectDate(self):
        selectedDate = self.selectedDate()
        self.dateClicked.emit(selectedDate) 

class LoginPage(QDialog, loginClass):
    def __init__(self):
        super().__init__()
        self.initWindow()        
        self.setHostAddress()


    def initWindow(self):
        self.setupUi(self)
        self.setWindowTitle("Login")
        self.loginBtn.clicked.connect(self.loginBtnClicked)

    def setHostAddress(self):
        try:
            # 첫 번째 네트워크 인터페이스의 IP 주소 가져오기
            iface = netifaces.interfaces()[1]
            addrs = netifaces.ifaddresses(iface)
            ip = addrs[netifaces.AF_INET][0]['addr']
            self.hostLine.setText(ip)
        except Exception as e:
            self.hostLine.setText("Unable to get IP")
    
    def loginBtnClicked(self):
        domainID = int(self.domainIDLine.text())
        os.environ['ROS_DOMAIN_ID'] = str(domainID)
        
        rp.init()
        self.storeNode = StoreNode()
        self.storeNodeThread = threading.Thread(target=rp.spin, args=(self.storeNode,))
        self.storeNodeThread.start()
    
        self.mainWindow = MainPage(self.storeNode)
        self.mainWindow.show()
        self.close()

class MainPage(QMainWindow, mainClass):
    def __init__(self, storeNode):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main")
        self.storeNode = storeNode
        self.initWindow()

    def initWindow(self):
        self.setSpinBoxesEnabled(False)
        self.initializeCalendar()
        self.robotManageBtn.clicked.connect(self.showRobotManagePage)
        self.modifyBtn.clicked.connect(self.modifyBtnClicked)
        self.handelqtSignal()
        self.updateData()       

    def handelqtSignal(self):
        self.storeNode.dailyTotalSales.connect(self.updateDailyTotalSales)
        self.storeNode.monthTotalSales.connect(self.updateMonthTotalSales)
        self.storeNode.stocks.connect(self.updateStocks)
        self.calendarWidget.dateClicked.connect(self.showDailySalesPage)

    def initializeCalendar(self):
        self.calendarContainer = self.findChild(QWidget, "calendarContainer")  # 디자이너에서 설정한 이름 사용
        self.calendarLayout = QVBoxLayout(self.calendarContainer)
        self.calendarWidget = Calendar(self.calendarContainer)
        self.calendarLayout.addWidget(self.calendarWidget)

    def setSpinBoxesEnabled(self, enabled):
        spinBoxes = [
            self.strawberrySpinBox,
            self.bananaSpinBox,
            self.chocolateSpinBox,
            self.affogatoSpinBox,
            self.toppingASpinBox,
            self.toppingBSpinBox,
            self.toppingCSpinBox
        ]
        for spinBox in spinBoxes:
            spinBox.setEnabled(enabled)

    def updateData(self):
        year = str(self.calendarWidget.yearShown())
        month = str(self.calendarWidget.monthShown())
        self.storeNode.requestDailyTotalSales(year, month)
        self.storeNode.requestMonthTotalSales(year, month)
        self.storeNode.requestStocks()
    
    def closeEvent(self, event):
        self.storeNode.destroy_node()
        rp.shutdown()
        event.accept()  # 창을 닫음

    @pyqtSlot(object)
    def updateDailyTotalSales(self, data):
        self.storeNode.get_logger().info(f"Updated Daily Total Sales: {data}")
        salesRecords = {}
        for record in data:
            date_str, total_sales = record.split(',')
            date = QDate.fromString(date_str, "yyyy-MM-dd")
            salesRecords[date] = f"{int(total_sales):,}원"

        self.calendarWidget.setDailyTotalSales(salesRecords)

    @pyqtSlot(object)
    def updateMonthTotalSales(self,data):
        self.storeNode.get_logger().info(f"Updated Monthly Total Sales: {data}")
        self.totalSalesLine.setText(f"{data:,}원")
        self.totalSalesLine.setAlignment(Qt.AlignRight)

    @pyqtSlot(object)
    def updateStocks(self, data):
        # print(data)

        spinBoxMapping = {
            "딸기" : self.strawberrySpinBox,
            "바나나" : self.bananaSpinBox,
            "초코" : self.chocolateSpinBox,
            "아포가토" : self.affogatoSpinBox,
            "토핑A" : self.toppingASpinBox,
            "토핑B" : self.toppingBSpinBox,
            "토핑C" : self.toppingCSpinBox,
        }

        for item in data.menu:
            if item.name in spinBoxMapping:
                spinBoxMapping[item.name].setValue(item.stock)

        # topping 항목 처리
        for item in data.topping:
            if item.name in spinBoxMapping:
                spinBoxMapping[item.name].setValue(item.stock)

    def modifyBtnClicked(self):
        text = self.modifyBtn.text()
        if text == "수정":
            self.setSpinBoxesEnabled(True)
            self.modifyBtn.setText("저장")
        elif text == "저장":
            self.modifyStock()
            self.setSpinBoxesEnabled(False)
            self.modifyBtn.setText("수정")
    
    def modifyStock(self):
        strawberryStock = self.strawberrySpinBox.value()
        bananaStock = self.bananaSpinBox.value()
        chocolateStock = self.chocolateSpinBox.value()
        affogatoStock = self.affogatoSpinBox.value()
        menuData = [
            StockInfo(name="딸기", stock=strawberryStock),
            StockInfo(name="바나나", stock=bananaStock),
            StockInfo(name="초코", stock=chocolateStock),
            StockInfo(name="아포가토", stock=affogatoStock)
        ]
        toppingAStock = self.toppingASpinBox.value()
        toppingBStock = self.toppingBSpinBox.value()
        toppingCStock = self.toppingCSpinBox.value()
        toppingData = [
            StockInfo(name="토핑A", stock=toppingAStock),
            StockInfo(name="토핑B", stock=toppingBStock),
            StockInfo(name="토핑C", stock=toppingCStock)
        ]
        self.storeNode.requestModifyStocks(menuData, toppingData)

    @pyqtSlot(QDate)
    def showDailySalesPage(self, date):
        self.dailySalesWindow = DailySalesPage(self.storeNode, date)
        self.dailySalesWindow.show()

    def showRobotManagePage(self):
        self.robotmanageWindow = RobotManagePage(self.storeNode)
        self.robotManageWindow.show()

class DailySalesPage(QDialog, dailySalesClass):
    def __init__(self, storeNode, date):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Daily Sales")
        self.date = date
        self.year = date.year()
        self.month = date.month()
        self.day = date.day()
        self.storeNode = storeNode
        self.initWindow()
    
    def initWindow(self):
        self.dailySalesRBtn.toggled.connect(self.showDailySales) # 일별 판매 기록 버튼 연결
        self.menuSalesRBtn.toggled.connect(self.showMenuSales) # 메뉴별 판매 기록 버튼 연결
        self.timeSalesRBtn.toggled.connect(self.showTimeSales)
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
        self.handelqtSignal()
        self.updateData()

    def updateData(self):
        self.updateSelectedDateLabel(self.date)
        self.storeNode.requestDailySales(self.year, self.month, self.day)
        self.storeNode.requestMenuDailySales(self.year, self.month, self.day)

    def updateSelectedDateLabel(self, date):
        self.dateLabel.setText(date.toString('yyyy년 MM월 dd일'))

    def handelqtSignal(self):
        self.storeNode.dailySales.connect(self.setDailySalesData)
        self.storeNode.menuDailySales.connect(self.setMenuSalesData)

    @pyqtSlot(object)
    def setDailySalesData(self, data):
        self.dailySales = data
        if self.dailySalesRBtn.isChecked():
            self.updateTable(data)

    @pyqtSlot(object)
    def setMenuSalesData(self, data):
        self.menuDailySales = data
        if self.menuSalesRBtn.isChecked():
            self.drawBarGraph(data)

    def updateTable(self, data):
        self.dailySalesTable.setRowCount(len(data))
        totalSales = 0

        for row_num, order_info in enumerate(data):
            items = [
                str(order_info.order_id),
                order_info.order_time,
                order_info.menu,
                order_info.topping,
                f"{order_info.quantity}개",
                f"{order_info.price}원"
            ]

            totalSales += order_info.price 

            for col_num, item in enumerate(items):
                table_item = QTableWidgetItem(item)
                table_item.setTextAlignment(Qt.AlignCenter)
                self.dailySalesTable.setItem(row_num, col_num, table_item)

        self.updateTotalSales(totalSales)

    def updateTotalSales(self, totalSales):
        self.totalSalesLine.setText(f"{totalSales:,}원")
        self.totalSalesLine.setAlignment(Qt.AlignRight)

    def showDailySales(self):
        if self.dailySalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(0)
            self.dailySalesTable.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
            self.updateTable(self.dailySales)

    def showMenuSales(self):
        if self.menuSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(1)
            self.drawBarGraph(self.menuDailySales)

    def showTimeSales(self):
        if self.timeSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(2)
    
    def drawBarGraph(self, data):
        sales_dict = {item.name: item.quantity for item in data}

        menuItems = list(sales_dict.keys())
        sales = list(sales_dict.values())

        # QImage 생성
        width, height = self.menuSalesGraph.width(), self.menuSalesGraph.height()
        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        # QPainter로 QImage에 그리기
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        # 그래프 영역 정의
        margin = 50
        graph_width = width - 2 * margin
        graph_height = height - 2 * margin

        # 최대 판매량 구하기
        max_sales = max(sales, default=1)

        # 각 메뉴에 대한 색상 정의
        colors = {
            '딸기': QColor('#FF6347'),  # 토마토 레드
            '바나나': QColor('#FFD700'),  # 골드
            '초코': QColor('#8B4513'),  # 새들 브라운
            '아포가토': QColor('#6F4E37')  # 커피 색
        }

        # 막대 그리기
        bar_width = graph_width / len(menuItems)
        for i, (menu, sale) in enumerate(zip(menuItems, sales)):
            x = margin + int(i * bar_width)
            y = height - margin - int(sale / max_sales * graph_height)
            painter.setBrush(colors[menu])
            painter.drawRect(int(x), int(y), int(bar_width * 0.8), int(height - margin - y))

            # 막대 위에 텍스트 추가
            painter.setPen(Qt.black)
            painter.setFont(QFont('Arial', 10))
            painter.drawText(int(x + bar_width * 0.4), int(y - 10), str(sale))

            # 메뉴 라벨 추가
            painter.setFont(QFont('Arial', 10, QFont.Bold))
            painter.drawText(int(x + bar_width * 0.4) - 10, int(height - margin + 20), menu)

        # 축 그리기
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
        painter.drawLine(margin, margin, margin, height - margin)  # Y 축

        # 제목 그리기
        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.drawText(int(width / 2 - 50), margin - 25, '메뉴별 일일 판매량')

        painter.end()

        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(image)
        self.menuSalesGraph.setPixmap(pixmap)

class RobotManagePage(QDialog, robotClass):
    def __init__(self, storeNode):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Robot Manage")
        self.storeNode = storeNode

    @pySlot(object)
    def updateJointTemperature(self, data):
        pass


def main(args=None):
    app = QApplication(sys.argv)
    


    loginWindow = LoginPage()
    loginWindow.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("Shutting down")

    rp.shutdown()

if __name__ == "__main__":
    main()
