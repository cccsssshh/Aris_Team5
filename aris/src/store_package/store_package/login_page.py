import os
import glob
import time
import netifaces
import threading
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIntValidator
import rclpy as rp
from store_package.main_page import MainPage
from store_package.store_node import StoreNode
from store_package.ui_loader import loginClass


class LoginPage(QDialog, loginClass):
    def __init__(self):
        super().__init__()
        self.initWindow()        
        self.setHostAddress()
        self.storeNode = None
        self.storeNodeThread = None
        int_validator = QIntValidator(0, 232)
        self.domainIDLine.setValidator(int_validator)

    def initWindow(self):
        self.setupUi(self)
        self.setWindowTitle("Login")
        self.loginBtn.clicked.connect(self.loginBtnClicked)
        self.cancelBtn.clicked.connect(self.cancelBtnClicked)

    def setHostAddress(self):
        try:
            iface = 'wlp4s0'
            addrs = netifaces.ifaddresses(iface)
            if netifaces.AF_INET in addrs:
                ip_address = addrs[netifaces.AF_INET][0]['addr']
                self.hostLine.setText(ip_address)
            else:
                self.hostLine.setText("No valid IP found for wlp4s0")
        except Exception as e:
            self.hostLine.setText("Unable to get IP")
            print(e)
        
    def loginBtnClicked(self):
        try:
            domainID = int(self.domainIDLine.text())
            if domainID < 0 or domainID > 232:
                raise ValueError

            os.environ['ROS_DOMAIN_ID'] = str(domainID)

            if self.storeNode is None:
                rp.init()
                self.storeNode = StoreNode()
                self.storeNodeThread = threading.Thread(target=rp.spin, args=(self.storeNode,))
                self.storeNodeThread.setDaemon(True)
                self.storeNodeThread.start()
                self.loginBtn.setEnabled(False)

                self.storeNode.allServicesAvailable.connect(self.MoveToMain)
        except ValueError:
            QMessageBox.warning(self, "Invalid Input", "Please enter a valid integer between 0 and 232 for the domain ID.")

    def cancelBtnClicked(self):
        if self.storeNode is not None:
            self.storeNode.timer.cancel()
            time.sleep(2)
            self.storeNode.destroy_node()
            rp.shutdown()
            self.storeNode = None
        if self.storeNodeThread is not None:
            self.storeNodeThread.join()
            self.storeNodeThread = None
            self.loginBtn.setEnabled(True)

    def MoveToMain(self):
        self.mainWindow = MainPage(self.storeNode)
        self.mainWindow.show()
        self.hide()
    
    def closeEvent(self, event):
        if self.storeNode is not None:
            self.storeNode.timer.cancel()
            time.sleep(2)
            self.storeNode.destroy_node()
            rp.shutdown()
        event.accept()