from PyQt5.QtWidgets import *
from PyQt5.QtCore import pyqtSlot
from store_package.ui_loader import robotClass

class RobotManagePage(QDialog, robotClass):
    def __init__(self, storeNode):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Robot Manage")
        self.storeNode = storeNode
        self.handleqtSignal()

    def handleqtSignal(self):
        self.storeNode.robotStatus.connect(self.updateRobotStatus)

    @pyqtSlot(list, list)
    def updateRobotStatus(self, joints, temperatures):
        print("Joints:", joints)
        print("Temperatures:", temperatures)

        self.joint1Line.setText(joints[0])
        self.joint2Line.setText(joints[1])
        self.joint3Line.setText(joints[2])
        self.joint4Line.setText(joints[3])
        self.joint5Line.setText(joints[4])
        self.joint6Line.setText(joints[5])

        self.temp1Line.setText(temperatures[0])
        self.temp2Line.setText(temperatures[1])
        self.temp3Line.setText(temperatures[2])
        self.temp4Line.setText(temperatures[3])
        self.temp5Line.setText(temperatures[4])
        self.temp6Line.setText(temperatures[5])