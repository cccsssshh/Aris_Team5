from PyQt5.QtWidgets import QCalendarWidget
from PyQt5.QtCore import pyqtSignal, QDate, Qt, QRect
from PyQt5.QtGui import QColor, QFont

class Calendar(QCalendarWidget):
    dateClicked = pyqtSignal(QDate)
    monthChanged = pyqtSignal(int, int) 

    def __init__(self, parent=None):
        super(Calendar, self).__init__(parent)
        self.salesRecords = {}
        self.initWindow()

    def initWindow(self):
        self.clicked.connect(self.selectDate)
        self.currentPageChanged.connect(self.onMonthChanged)

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
    
    def onMonthChanged(self, year, month):
        self.monthChanged.emit(year, month)