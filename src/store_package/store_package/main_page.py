import os
import glob
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import QDate, Qt, pyqtSlot

import rclpy as rp
from interface_package.msg import StockInfo


from store_package.ui_components import Calendar
from store_package.daily_sales_page import DailySalesPage
from store_package.robot_manage_page import RobotManagePage
from store_package.ui_loader import mainClass

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
        self.calendarWidget.monthChanged.connect(self.showMonthlySales)

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

    @pyqtSlot(int, int)
    def showMonthlySales(self, year, month):
        self.storeNode.requestDailyTotalSales(str(year), str(month))
        self.storeNode.requestMonthTotalSales(str(year), str(month))

    def showRobotManagePage(self):
        self.robotManageWindow = RobotManagePage(self.storeNode)
        self.robotManageWindow.show()


