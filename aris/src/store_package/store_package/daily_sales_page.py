import os
import glob
from PyQt5.QtWidgets import *
from PyQt5 import uic
from PyQt5.QtCore import Qt, QRect,  pyqtSlot, QPoint
from PyQt5.QtGui import QColor, QFont, QPainter, QPixmap, QImage, QPen, QPolygon


from store_package.ui_loader import dailySalesClass

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
        self.dailySales = None
        self.initWindow()
    
    def initWindow(self):
        self.dailySalesRBtn.toggled.connect(self.showDailySales)
        self.menuSalesRBtn.toggled.connect(self.showMenuSales)
        self.hourlySalesRBtn.toggled.connect(self.showHourlySales)
        self.menuToppingSalesRBtn.toggled.connect(self.showmenuToppingSales)
        self.ageSalesRBtn.toggled.connect(self.showAgeSales)
        self.dailySalesRBtn.setChecked(True)
        self.dailySalesTable.verticalHeader().setVisible(False)
        self.handelqtSignal()
        self.updateData()

    def updateData(self):
        self.updateSelectedDateLabel(self.date)
        self.storeNode.requestDailySales(self.year, self.month, self.day)
        self.storeNode.requestMenuDailySales(self.year, self.month, self.day)
        self.storeNode.requestHourlySales(self.year, self.month, self.day)
        self.storeNode.requestMenuToppingSales(self.year, self.month, self.day)
        self.storeNode.requestAgeSales(self.year, self.month, self.day)

    def updateSelectedDateLabel(self, date):
        self.dateLabel.setText(date.toString('yyyy년 MM월 dd일'))

    def handelqtSignal(self):
        self.storeNode.dailySales.connect(self.setDailySalesData)
        self.storeNode.menuDailySales.connect(self.setMenuSalesData)
        self.storeNode.hourlySales.connect(self.setHourlySalesData)
        self.storeNode.ageSales.connect(self.setAgeSalesData)
        self.storeNode.menuToppingSales.connect(self.setMenuToppingSalesData)

    @pyqtSlot(object)
    def setDailySalesData(self, data):
        self.dailySales = data
        if self.dailySalesRBtn.isChecked():
            self.updateTable(data)

    @pyqtSlot(object)
    def setMenuSalesData(self, data):
        self.menuDailySales = data
        if self.menuSalesRBtn.isChecked():
            self.drawDailyBarGraph(data)

    @pyqtSlot(object)
    def setHourlySalesData(self, data):
        self.hourlySales = data
        if self.hourlySalesRBtn.isChecked():
            self.drawLineGraph(data)

    @pyqtSlot(list, list, list)
    def setMenuToppingSalesData(self, menu_names, topping_names, counts):
        self.menuNames = menu_names
        self.toppingNames = topping_names
        self.counts = counts
        if self.menuToppingSalesRBtn.isChecked():
            self.drawMenuToppingGridGraph(self, menu_names, topping_names, counts)

    @pyqtSlot(list, list)
    def setAgeSalesData(self, age_groups, age_group_sales):
        self.ageGroups = age_groups
        self.ageGroupSales = age_group_sales
        if self.ageSalesRBtn.isChecked():
            self.drawLineGraph(age_groups, age_group_sales)

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
            if self.dailySales:
                self.updateTable(self.dailySales)

    def showMenuSales(self):
        if self.menuSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(1)
            self.drawDailyBarGraph(self.menuDailySales)

    def showHourlySales(self):
        if self.hourlySalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(2)
            self.drawLineGraph(self.hourlySales)

    def showmenuToppingSales(self):
        if self.menuToppingSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(3)
            self.drawMenuToppingGridGraph(self.menuNames, self.toppingNames, self.counts)

    def showAgeSales(self):
        if self.ageSalesRBtn.isChecked():
            self.stackedWidget.setCurrentIndex(4)
            self.drawAgeBarGraph(self.ageGroups, self.ageGroupSales)

    def drawDailyBarGraph(self, data):
        all_menu_items = ['딸기', '바나나', '초코', '아포가토']

        sales_dict = {item.name: item.quantity for item in data}
        sales_dict = {menu: sales_dict.get(menu, 0) for menu in all_menu_items}

        menuItems = list(sales_dict.keys())
        sales = list(sales_dict.values())

        width, height = self.menuSalesGraph.width(), self.menuSalesGraph.height()
        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        margin = 50
        graph_width = width - 2 * margin
        graph_height = height - 2 * margin
        max_sales = max(sales, default=1)

        y_max = max(max_sales, 9)

        colors = {
            '딸기': QColor('#FF6347'),
            '바나나': QColor('#FFD700'), 
            '초코': QColor('#8B4513'),
            '아포가토': QColor('#6F4E37')
        }

        bar_width = graph_width / len(menuItems)
        for i, (menu, sale) in enumerate(zip(menuItems, sales)):
            x = margin + int(i * bar_width)
            y = height - margin - int(sale / y_max * graph_height)
            painter.setBrush(colors[menu])
            painter.drawRect(int(x), int(y), int(bar_width * 0.8), int(height - margin - y))

            painter.setPen(Qt.black)
            painter.setFont(QFont('Arial', 10))
            painter.drawText(int(x + bar_width * 0.4), int(y - 10), str(sale))

            painter.setFont(QFont('Arial', 10, QFont.Bold))
            painter.drawText(int(x + bar_width * 0.4) - 10, int(height - margin + 20), menu)

        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
        painter.drawLine(margin, margin, margin, height - margin)  # Y 축

        if y_max <= 9:
            num_y_ticks = y_max
            y_tick_distance = graph_height / num_y_ticks
            for i in range(num_y_ticks + 1):
                y = height - margin - int(i * y_tick_distance)
                sales_value = i
                painter.drawLine(margin - 5, y, margin, y)
                painter.drawText(margin - 40, y + 5, f"{sales_value}")
        else:
            num_y_ticks = 5
            y_tick_distance = graph_height / num_y_ticks
            for i in range(num_y_ticks + 1):
                y = height - margin - int(i * y_tick_distance)
                sales_value = int(y_max * (i / num_y_ticks))
                painter.drawLine(margin - 5, y, margin, y)
                painter.drawText(margin - 40, y + 5, f"{sales_value}")

        painter.setFont(QFont('Arial', 12))
        painter.drawText(width // 2, height - margin + 40, '메뉴명')

        painter.setFont(QFont('Arial', 12))
        painter.drawText(margin - 40, margin - 10, '판매량 (개)')

        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.drawText(int(width / 2 - 75), margin - 25, '메뉴별 일일 판매량')

        painter.end()

        pixmap = QPixmap.fromImage(image)
        self.menuSalesGraph.setPixmap(pixmap)

    def drawLineGraph(self, data):
        sales_dict = {int(item.name): item.quantity for item in data}

        all_hours = list(range(24))
        sales = [sales_dict.get(hour, 0) for hour in all_hours]

        width, height = self.hourlySalesGraph.width(), self.hourlySalesGraph.height()
        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        margin = 50
        graph_width = width - 2 * margin
        graph_height = height - 2 * margin

        max_sales = max(sales, default=1)

        point_distance = graph_width / (len(all_hours) - 1)
        points = []
        for i, sale in enumerate(sales):
            x = margin + int(i * point_distance)
            y = height - margin - int(sale / max_sales * graph_height)
            points.append(QPoint(x, y))

        painter.setPen(QPen(Qt.black, 2))
        painter.drawPolyline(QPolygon(points))

        painter.setBrush(Qt.green)
        for point in points:
            painter.drawEllipse(point, 5, 5)

        painter.setPen(Qt.black)
        painter.setFont(QFont('Arial', 10))
        for i, point in enumerate(points):
            painter.drawText(point.x() - 10, point.y() - 10, str(sales[i]))

        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
        painter.drawLine(margin, margin, margin, height - margin)  # Y 축

        for i in range(len(all_hours)):
            x = margin + int(i * point_distance)
            painter.drawLine(x, height - margin, x, height - margin + 5)
            painter.drawText(x - 10, height - margin + 20, f"{all_hours[i]}")

        num_y_ticks = 5
        y_tick_distance = graph_height / num_y_ticks
        for i in range(num_y_ticks + 1):
            y = height - margin - int(i * y_tick_distance)
            sales_value = int(max_sales * (i / num_y_ticks))
            painter.drawLine(margin - 5, y, margin, y)
            painter.drawText(margin - 40, y + 5, f"{sales_value}")

        painter.setFont(QFont('Arial', 12))
        painter.drawText(width - margin - 30, height - margin + 40, '시간 (시)')  # 약간 왼쪽으로 이동

        painter.setFont(QFont('Arial', 12))
        painter.drawText(margin - 40, margin - 10, '판매량 (개)')

        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.drawText(int(width / 2 - 75), margin - 25, '시간대별 아이스크림 판매량')

        painter.end()

        pixmap = QPixmap.fromImage(image)
        self.hourlySalesGraph.setPixmap(pixmap)

    def drawMenuToppingGridGraph(self, menu_names, topping_names, counts):
        size = self.menuToppingGraph.size()
        width, height = size.width(), size.height()

        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        left_margin = 80
        right_margin = 20
        top_margin = 50
        bottom_margin = 50
        graph_width = width - left_margin - right_margin
        graph_height = height - top_margin - bottom_margin

        unique_menus = sorted(set(menu_names))
        unique_toppings = sorted(set(topping_names))
        num_menus = len(unique_menus)
        num_toppings = len(unique_toppings)

        cell_width = graph_width / num_menus
        cell_height = graph_height / num_toppings

        min_count = min(counts) if counts else 0
        max_count = max(counts) if counts else 1 

        nonzero_counts = [count for count in counts if count > 0]
        min_nonzero_count = min(nonzero_counts) if nonzero_counts else 0
        max_nonzero_count = max(nonzero_counts) if nonzero_counts else max_count

        def get_color_for_count(count, min_nonzero_count, max_nonzero_count):
            color_start = QColor('#FFFFFF')
            color_end = QColor('#77FF77')

            if count == 0:
                return QColor(255, 255, 255, 0)

            if max_nonzero_count > min_nonzero_count:
                ratio = (count - min_nonzero_count) / (max_nonzero_count - min_nonzero_count)
            else:
                ratio = 0

            red = int(color_end.red() * ratio + color_start.red() * (1 - ratio))
            green = int(color_end.green() * ratio + color_start.green() * (1 - ratio))
            blue = int(color_end.blue() * ratio + color_start.blue() * (1 - ratio))
            
            return QColor(red, green, blue)

        counts_dict = {(menu, topping): count for (menu, topping), count in zip(zip(menu_names, topping_names), counts)}

        for i, menu in enumerate(unique_menus):
            for j, topping in enumerate(unique_toppings):
                count = counts_dict.get((menu, topping), 0)

                x = left_margin + i * cell_width
                y = top_margin + j * cell_height
                cell_rect = QRect(int(x), int(y), int(cell_width), int(cell_height))

                cell_color = get_color_for_count(count, min_nonzero_count, max_nonzero_count)
                painter.setBrush(cell_color)
                painter.drawRect(cell_rect)

                painter.setPen(QPen(QColor('#000000'), 1))
                painter.drawRect(cell_rect)

                painter.setPen(Qt.black)
                font = QFont('Arial', int(min(cell_width, cell_height) / 10)) 
                painter.setFont(font)
                text = f"{count}" if count > 0 else ""
                painter.drawText(cell_rect, Qt.AlignCenter, text)

        painter.setFont(QFont('Arial', 10))
        for i, menu in enumerate(unique_menus):
            x = left_margin + (i + 0.5) * cell_width
            painter.save()
            painter.translate(x, height - bottom_margin + 20)
            painter.drawText(0, 0, menu)
            painter.restore()

        painter.setFont(QFont('Arial', 10)) 
        for j, topping in enumerate(unique_toppings):
            y = top_margin + (j + 0.5) * cell_height
            painter.drawText(left_margin - 70, int(y), topping)

        painter.setFont(QFont('Arial', 12, QFont.Bold))
        painter.drawText(int(width / 2 - 80), top_margin - 20, '메뉴-토핑별 판매량')

        painter.end()

        pixmap = QPixmap.fromImage(image)
        self.menuToppingGraph.setPixmap(pixmap)

    def drawAgeBarGraph(self, age_groups, age_group_sales):
            size = self.ageGraph.size()
            width, height = size.width(), size.height()

            image = QImage(width, height, QImage.Format_ARGB32)
            image.fill(Qt.white)

            painter = QPainter(image)
            painter.setRenderHint(QPainter.Antialiasing)

            margin = 50
            graph_width = width - 2 * margin
            graph_height = height - 2 * margin

            max_sales = max(age_group_sales, default=1)

            y_max = max(max_sales, 9)

            colors = {
                '00-09': QColor('#77FF77'),
                '10-19': QColor('#77FF77'),
                '20-29': QColor('#77FF77'),
                '30-39': QColor('#77FF77'),
                '40-49': QColor('#77FF77'),
                '50-59': QColor('#77FF77'),
                '60+': QColor('#77FF77')
            }

            bar_width = graph_width / len(age_groups)
            for i, (age_group, sales) in enumerate(zip(age_groups, age_group_sales)):
                x = margin + int(i * bar_width)
                y = height - margin - int(sales / y_max * graph_height)
                painter.setBrush(colors.get(age_group, QColor('#A9A9A9')))
                painter.drawRect(int(x), int(y), int(bar_width * 0.8), int(height - margin - y))

                painter.setPen(Qt.black)
                painter.setFont(QFont('Arial', 10))
                painter.drawText(int(x + bar_width * 0.4), int(y - 10), str(sales))

                painter.setFont(QFont('Arial', 10, QFont.Bold))
                painter.drawText(int(x + bar_width * 0.4) - 10, int(height - margin + 20), age_group)

            pen = QPen(Qt.black, 2)
            painter.setPen(pen)
            painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
            painter.drawLine(margin, margin, margin, height - margin)  # Y 축

            if y_max <= 9:
                num_y_ticks = y_max
                y_tick_distance = graph_height / num_y_ticks
                for i in range(num_y_ticks + 1):
                    y = height - margin - int(i * y_tick_distance)
                    sales_value = i
                    painter.drawLine(margin - 5, y, margin, y)
                    painter.drawText(margin - 40, y + 5, f"{sales_value}")
            else:
                num_y_ticks = 5
                y_tick_distance = graph_height / num_y_ticks
                for i in range(num_y_ticks + 1):
                    y = height - margin - int(i * y_tick_distance)
                    sales_value = int(y_max * (i / num_y_ticks))
                    painter.drawLine(margin - 5, y, margin, y)
                    painter.drawText(margin - 40, y + 5, f"{sales_value}")

            painter.setFont(QFont('Arial', 12))
            painter.drawText(width // 2, height - margin + 40, '나이대')

            painter.setFont(QFont('Arial', 12))
            painter.drawText(margin - 40, margin - 10, '판매량 (개)')

            painter.setFont(QFont('Arial', 14, QFont.Bold))
            painter.drawText(int(width / 2 - 75), margin - 25, '나이대별 판매량')

            painter.end()

            pixmap = QPixmap.fromImage(image)
            self.ageGraph.setPixmap(pixmap)