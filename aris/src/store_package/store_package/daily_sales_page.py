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
        self.dailySalesRBtn.toggled.connect(self.showDailySales) # 일별 판매 기록 버튼 연결
        self.menuSalesRBtn.toggled.connect(self.showMenuSales) # 메뉴별 판매 기록 버튼 연결
        self.hourlySalesRBtn.toggled.connect(self.showHourlySales)
        self.menuToppingSalesRBtn.toggled.connect(self.showmenuToppingSales)
        self.ageSalesRBtn.toggled.connect(self.showAgeSales)
        self.dailySalesRBtn.setChecked(True) # 일별 판매 기록 버튼 활성화
        self.dailySalesTable.verticalHeader().setVisible(False) #일별 판매 기록 테이블의 왼쪽의 번호 비활성화
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
        # 모든 메뉴 아이템을 정의
        all_menu_items = ['딸기', '바나나', '초코', '아포가토']

        # 판매량 데이터를 딕셔너리로 변환
        sales_dict = {item.name: item.quantity for item in data}

        # 모든 메뉴 아이템을 포함하여 판매량을 0으로 설정
        sales_dict = {menu: sales_dict.get(menu, 0) for menu in all_menu_items}

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

        # Y축 최대값 설정 (9 이하일 경우 9로 설정, 그 이상일 경우 max_sales로 설정)
        y_max = max(max_sales, 9)

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
            y = height - margin - int(sale / y_max * graph_height)
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

        # Y축에 표시선과 레이블 추가
        if y_max <= 9:
            # 9 이하일 경우 1씩 증가
            num_y_ticks = y_max
            y_tick_distance = graph_height / num_y_ticks
            for i in range(num_y_ticks + 1):
                y = height - margin - int(i * y_tick_distance)
                sales_value = i
                painter.drawLine(margin - 5, y, margin, y)
                painter.drawText(margin - 40, y + 5, f"{sales_value}")
        else:
            # 그 이상일 경우 유동적으로 설정
            num_y_ticks = 5
            y_tick_distance = graph_height / num_y_ticks
            for i in range(num_y_ticks + 1):
                y = height - margin - int(i * y_tick_distance)
                sales_value = int(y_max * (i / num_y_ticks))
                painter.drawLine(margin - 5, y, margin, y)
                painter.drawText(margin - 40, y + 5, f"{sales_value}")

        # X축 제목 추가
        painter.setFont(QFont('Arial', 12))
        painter.drawText(width // 2, height - margin + 40, '메뉴명')

        # Y축 제목 추가
        painter.setFont(QFont('Arial', 12))
        painter.drawText(margin - 40, margin - 10, '판매량 (개)')

        # 그래프 제목 그리기
        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.drawText(int(width / 2 - 75), margin - 25, '메뉴별 일일 판매량')

        painter.end()

        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(image)
        self.menuSalesGraph.setPixmap(pixmap)

    def drawLineGraph(self, data):
        # name 속성을 사용하여 시간대별 판매량을 저장
        sales_dict = {int(item.name): item.quantity for item in data}

        # 모든 시간대를 포함하도록 설정
        all_hours = list(range(24))
        sales = [sales_dict.get(hour, 0) for hour in all_hours]

        # QImage 생성
        width, height = self.hourlySalesGraph.width(), self.hourlySalesGraph.height()
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

        # 시간대별 판매량 꺾은선 그리기
        point_distance = graph_width / (len(all_hours) - 1)
        points = []
        for i, sale in enumerate(sales):
            x = margin + int(i * point_distance)
            y = height - margin - int(sale / max_sales * graph_height)
            points.append(QPoint(x, y))

        painter.setPen(QPen(Qt.black, 2))  # 선 색상을 검은색으로 설정
        painter.drawPolyline(QPolygon(points))

        # 각 점에 원 그리기
        painter.setBrush(Qt.green)
        for point in points:
            painter.drawEllipse(point, 5, 5)

        # 각 점에 텍스트 추가
        painter.setPen(Qt.black)
        painter.setFont(QFont('Arial', 10))
        for i, point in enumerate(points):
            painter.drawText(point.x() - 10, point.y() - 10, str(sales[i]))

        # 축 그리기
        pen = QPen(Qt.black, 2)
        painter.setPen(pen)
        painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
        painter.drawLine(margin, margin, margin, height - margin)  # Y 축

        # X축에 표시선과 레이블 추가
        for i in range(len(all_hours)):
            x = margin + int(i * point_distance)
            painter.drawLine(x, height - margin, x, height - margin + 5)
            painter.drawText(x - 10, height - margin + 20, f"{all_hours[i]}")

        # Y축에 표시선과 레이블 추가
        num_y_ticks = 5
        y_tick_distance = graph_height / num_y_ticks
        for i in range(num_y_ticks + 1):
            y = height - margin - int(i * y_tick_distance)
            sales_value = int(max_sales * (i / num_y_ticks))
            painter.drawLine(margin - 5, y, margin, y)
            painter.drawText(margin - 40, y + 5, f"{sales_value}")

        # X축 제목 추가
        painter.setFont(QFont('Arial', 12))
        painter.drawText(width - margin - 30, height - margin + 40, '시간 (시)')  # 약간 왼쪽으로 이동

        # Y축 제목 추가
        painter.setFont(QFont('Arial', 12))
        painter.drawText(margin - 40, margin - 10, '판매량 (개)')

        # 그래프 제목 그리기
        painter.setFont(QFont('Arial', 14, QFont.Bold))
        painter.drawText(int(width / 2 - 75), margin - 25, '시간대별 아이스크림 판매량')

        painter.end()

        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(image)
        self.hourlySalesGraph.setPixmap(pixmap)

    def drawMenuToppingGridGraph(self, menu_names, topping_names, counts):
        # QLabel의 크기 가져오기
        size = self.menuToppingGraph.size()
        width, height = size.width(), size.height()

        # QImage 생성
        image = QImage(width, height, QImage.Format_ARGB32)
        image.fill(Qt.white)

        # QPainter로 QImage에 그리기
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)

        # 그래프 영역 정의
        left_margin = 80
        right_margin = 20
        top_margin = 50
        bottom_margin = 50
        graph_width = width - left_margin - right_margin
        graph_height = height - top_margin - bottom_margin

        # 메뉴와 토핑의 수
        unique_menus = sorted(set(menu_names))
        unique_toppings = sorted(set(topping_names))
        num_menus = len(unique_menus)
        num_toppings = len(unique_toppings)

        # 셀 크기 정의
        cell_width = graph_width / num_menus
        cell_height = graph_height / num_toppings

        # 판매량 범위 계산
        min_count = min(counts) if counts else 0
        max_count = max(counts) if counts else 1  # 최대값이 0일 때 대비를 위한 기본값

        # 판매량이 0이 아닌 최소값을 찾습니다.
        nonzero_counts = [count for count in counts if count > 0]
        min_nonzero_count = min(nonzero_counts) if nonzero_counts else 0
        max_nonzero_count = max(nonzero_counts) if nonzero_counts else max_count

        def get_color_for_count(count, min_nonzero_count, max_nonzero_count):
            # 색상 범위 설정
            color_start = QColor('#FFFFFF')  # 흰색 (판매량이 최소일 때의 색상)
            color_end = QColor('#77FF77')    # 연두색 (판매량이 최대일 때의 색상)

            if count == 0:
                return QColor(255, 255, 255, 0)  # 완전히 투명한 흰색

            # 판매량 비율 계산
            if max_nonzero_count > min_nonzero_count:
                ratio = (count - min_nonzero_count) / (max_nonzero_count - min_nonzero_count)
            else:
                ratio = 0

            # 색상 보간 (비율에 따라 색상 조정)
            red = int(color_end.red() * ratio + color_start.red() * (1 - ratio))
            green = int(color_end.green() * ratio + color_start.green() * (1 - ratio))
            blue = int(color_end.blue() * ratio + color_start.blue() * (1 - ratio))
            
            return QColor(red, green, blue)

        # 판매량 데이터를 매핑
        counts_dict = {(menu, topping): count for (menu, topping), count in zip(zip(menu_names, topping_names), counts)}

        # 그리드 그리기
        for i, menu in enumerate(unique_menus):
            for j, topping in enumerate(unique_toppings):
                count = counts_dict.get((menu, topping), 0)

                x = left_margin + i * cell_width
                y = top_margin + j * cell_height
                cell_rect = QRect(int(x), int(y), int(cell_width), int(cell_height))

                # 셀 배경색
                cell_color = get_color_for_count(count, min_nonzero_count, max_nonzero_count)
                painter.setBrush(cell_color)
                painter.drawRect(cell_rect)

                # 셀 테두리 색상
                painter.setPen(QPen(QColor('#000000'), 1))  # 테두리 두께를 1로 줄임
                painter.drawRect(cell_rect)

                # 텍스트 추가
                painter.setPen(Qt.black)
                font = QFont('Arial', int(min(cell_width, cell_height) / 10))  # 동적 폰트 크기
                painter.setFont(font)
                text = f"{count}" if count > 0 else ""
                painter.drawText(cell_rect, Qt.AlignCenter, text)

        # X축 제목 추가 (메뉴)
        painter.setFont(QFont('Arial', 10))  # 폰트 크기 줄임
        for i, menu in enumerate(unique_menus):
            x = left_margin + (i + 0.5) * cell_width
            painter.save()
            painter.translate(x, height - bottom_margin + 20)
            painter.drawText(0, 0, menu)
            painter.restore()

        # Y축 제목 추가 (토핑)
        painter.setFont(QFont('Arial', 10))  # 폰트 크기 줄임
        for j, topping in enumerate(unique_toppings):
            y = top_margin + (j + 0.5) * cell_height
            painter.drawText(left_margin - 70, int(y), topping)

        # 그래프 제목 추가
        painter.setFont(QFont('Arial', 12, QFont.Bold))  # 폰트 크기 줄임
        painter.drawText(int(width / 2 - 80), top_margin - 20, '메뉴-토핑별 판매량')

        painter.end()

        # QImage를 QPixmap으로 변환하여 QLabel에 설정
        pixmap = QPixmap.fromImage(image)
        self.menuToppingGraph.setPixmap(pixmap)

    def drawAgeBarGraph(self, age_groups, age_group_sales):
            # QLabel의 크기 가져오기
            size = self.ageGraph.size()
            width, height = size.width(), size.height()

            # QImage 생성
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
            max_sales = max(age_group_sales, default=1)

            # Y축 최대값 설정
            y_max = max(max_sales, 9)

            # 나이대 색상 정의 (파스텔톤 초록색)
            colors = {
                '00-09': QColor('#77FF77'),  # 파스텔 초록색
                '10-19': QColor('#77FF77'),  # 파스텔 초록색
                '20-29': QColor('#77FF77'),  # 파스텔 초록색
                '30-39': QColor('#77FF77'),  # 파스텔 초록색
                '40-49': QColor('#77FF77'),  # 파스텔 초록색
                '50-59': QColor('#77FF77'),  # 파스텔 초록색
                '60+': QColor('#77FF77')  # 파스텔 초록색
            }

            # 막대 그리기
            bar_width = graph_width / len(age_groups)
            for i, (age_group, sales) in enumerate(zip(age_groups, age_group_sales)):
                x = margin + int(i * bar_width)
                y = height - margin - int(sales / y_max * graph_height)
                painter.setBrush(colors.get(age_group, QColor('#A9A9A9')))  # 기본 회색
                painter.drawRect(int(x), int(y), int(bar_width * 0.8), int(height - margin - y))

                # 막대 위에 텍스트 추가
                painter.setPen(Qt.black)
                painter.setFont(QFont('Arial', 10))
                painter.drawText(int(x + bar_width * 0.4), int(y - 10), str(sales))

                # 나이대 라벨 추가
                painter.setFont(QFont('Arial', 10, QFont.Bold))
                painter.drawText(int(x + bar_width * 0.4) - 10, int(height - margin + 20), age_group)

            # 축 그리기
            pen = QPen(Qt.black, 2)
            painter.setPen(pen)
            painter.drawLine(margin, height - margin, width - margin, height - margin)  # X 축
            painter.drawLine(margin, margin, margin, height - margin)  # Y 축

            # Y축에 표시선과 레이블 추가
            if y_max <= 9:
                # 9 이하일 경우 1씩 증가
                num_y_ticks = y_max
                y_tick_distance = graph_height / num_y_ticks
                for i in range(num_y_ticks + 1):
                    y = height - margin - int(i * y_tick_distance)
                    sales_value = i
                    painter.drawLine(margin - 5, y, margin, y)
                    painter.drawText(margin - 40, y + 5, f"{sales_value}")
            else:
                # 그 이상일 경우 유동적으로 설정
                num_y_ticks = 5
                y_tick_distance = graph_height / num_y_ticks
                for i in range(num_y_ticks + 1):
                    y = height - margin - int(i * y_tick_distance)
                    sales_value = int(y_max * (i / num_y_ticks))
                    painter.drawLine(margin - 5, y, margin, y)
                    painter.drawText(margin - 40, y + 5, f"{sales_value}")

            # X축 제목 추가
            painter.setFont(QFont('Arial', 12))
            painter.drawText(width // 2, height - margin + 40, '나이대')

            # Y축 제목 추가
            painter.setFont(QFont('Arial', 12))
            painter.drawText(margin - 40, margin - 10, '판매량 (개)')

            # 그래프 제목 그리기
            painter.setFont(QFont('Arial', 14, QFont.Bold))
            painter.drawText(int(width / 2 - 75), margin - 25, '나이대별 판매량')

            painter.end()

            # QImage를 QPixmap으로 변환하여 QLabel에 설정
            pixmap = QPixmap.fromImage(image)
            self.ageGraph.setPixmap(pixmap)