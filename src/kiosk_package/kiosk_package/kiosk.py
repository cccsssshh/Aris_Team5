import sys
import os
from ament_index_python.packages import get_package_share_directory
from interface_package.srv import IceRobot, OrderRecord, RestQuantity
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5.QtCore import *
from PyQt5 import uic
import speech_recognition as sr
import pyaudio
import threading
import cv2
import numpy as np
import mediapipe as mp
from deeplearning_model import GestureModel, AgeModel
import threading
import torch
from deepface import DeepFace

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
package_share_directory = get_package_share_directory('kiosk_package')
ui_file_path = os.path.join(package_share_directory, 'ui/kiosk.ui')
image_folder_path = os.path.join(package_share_directory, 'ui/image/')

first_class = uic.loadUiType(ui_file_path)[0]

# Kiosk - Robot, DB
menu = None  # 메뉴

# Kiosk - Robot
order_num = None  # 주문 개수
shaking = None  # hello
tracking = None  # serving
half = None  # 반반 주문

# Kiosk - DB
order_number = 0  # 주문번호
date = None  # 주문날짜, 시간
topping = None  # 토핑A
price = 0  # 총 금액
quantity = 0  # 총 수량
gender = None  # 성별
age = None  # 연령대

menu_price = 0  # 주문 시에 쓰는 임시 변수
orders = []  # 주문 내역 임시 저장 변수 (리스트)

class MinimalClientAsync(Node, QObject):
    result_ready = pyqtSignal(int)
    restquantity = pyqtSignal(object)

    def __init__(self):
        super().__init__('minimal_client_async')
        QObject.__init__(self)
        self.cli = self.create_client(OrderRecord, 'OrderRecord')
        self.cli2 = self.create_client(RestQuantity, 'RestQuantity')
        self.cli3 = self.create_client(IceRobot, "IceRobot")
        self.get_logger().info('node start')
        
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        # while not self.cli2.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        # while not self.cli3.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')

        self.req = OrderRecord.Request()
        self.req2 = RestQuantity.Request()
        self.req3 = IceRobot.Request()

    @pyqtSlot(int, str, str, str, int, int, str, str)
    def send_request(self, order_number, date, menu, topping, price, quantity, gender, age):
        self.req.order_number = order_number
        self.req.date = date
        self.req.menu = menu
        self.req.topping = topping
        self.req.price = price
        self.req.quantity = quantity
        self.req.gender = gender
        self.req.age = age
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    def send_request2(self):
        self.get_logger().info("connected")
        self.future2 = self.cli2.call_async(self.req2)
        self.future2.add_done_callback(self.future2_callback)
    
    def future2_callback(self, future2):
        try:
            response = future2.result()
            self.get_logger().info(f'Result: {response.strawberry}, {response.chocolate}, {response.banana}, {response.affogato}')
            
            signals = []
            max_value = max(response.strawberry, response.chocolate, response.banana, response.affogato)
            min_value = min(response.strawberry, response.chocolate, response.banana, response.affogato)
            
            if max_value == response.strawberry and min_value <= 3:
                signals.append('strawberry_max')
            if max_value == response.chocolate and min_value <= 3:
                signals.append('chocolate_max')
            if max_value == response.banana and min_value <= 3:
                signals.append('banana_max')
            if max_value == response.affogato and min_value <= 3:
                signals.append('affogato_max')
            if response.strawberry == 0:
                signals.append('strawberry_zero')
            if response.chocolate == 0:
                signals.append('chocolate_zero')
            if response.banana == 0:
                signals.append('banana_zero')
            if response.affogato == 0:
                signals.append('affogato_zero')
            
            if signals:
                self.restquantity.emit(signals)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
    
    @pyqtSlot(str, int, str, str, str)
    def send_request3(self, menu, order_num, tracking, shaking, half):
        self.req3.menu = menu
        self.req3.order_num = order_num
        self.req3.tracking = tracking
        self.req3.shaking = shaking
        self.req3.half = half
        self.future3 = self.cli3.call_async(self.req3)
        self.future3.add_done_callback(self.future3_callback)

    def future3_callback(self, future3):
        try:
            response = future3.result()
            self.get_logger().info(f'Result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

class FirstClass(QMainWindow,first_class):
    """오픈화면 & 메인화면 창"""
    clicked = pyqtSignal()

    def add_page_mouse_press(self, event):
        """오픈화면 누르면 발생하는 이벤트"""
        self.stackedWidget.setCurrentWidget(self.main_page)
    
    # 이미지 처리 함수 ####################################################################################################
    def set_ad_image(self):
        self.ad_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'ARIS.jpg')).scaled(QSize(768, 1024)))  # 첫번째 이미지

    ####################################################################################################################

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main Page")

        self.init_ros2()

        self.ros2_client_worker.get_logger().info('ros2 client worker start')

        # 오픈화면 #######################################################################################################
        self.stackedWidget.setCurrentIndex(0)  # 시작할때 화면은 오픈 페이지로 설정
        self.set_ad_image()  
        self.setWindowFlags(Qt.FramelessWindowHint) # 프레임 지우기
        self.move(10,30) #창이동

        # 페이지 이동 및 타이머 시작
        self.ad_label.mousePressEvent = self.start_main_page # 페이지 이동)

        # 메인화면 #######################################################################################################
        self.berry_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'berry.webp')).scaled(QSize(377, 198)))
        self.choco_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'choco.webp')).scaled(QSize(377, 198)))
        self.banana_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'banana.webp')).scaled(QSize(377, 198)))
        self.affogato_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'affogato.jpg')).scaled(QSize(377, 198)))
        self.serving_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'serving.jpg')).scaled(QSize(243, 214)))
        self.hello_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'hand.jpg')).scaled(QSize(243, 214)))
        self.half_label.setPixmap(QPixmap(os.path.join(image_folder_path, 'half_half.jpg')).scaled(QSize(243, 214)))

        self.berry_recommend_label.hide()
        self.berry_soldout_label.hide()
        self.choco_recommend_label.hide()
        self.choco_soldout_label.hide()
        self.banana_recommend_label.hide()
        self.banana_soldout_label.hide()
        self.affogato_recommend_label.hide()
        self.affogato_soldout_label.hide()

        self.ros2_client_worker.restquantity.connect(self.recommend_soldout)

        self.menu_pushButton.clicked.connect(lambda: self.check_current_page(1))
        self.option_pushButton.clicked.connect(lambda: self.check_current_page(2))
        self.webcam_pushButton.clicked.connect(lambda: self.check_current_page(3))

        self.berry_plus_pushButton.clicked.connect(lambda: self.Berry_Add())
        self.berry_minus_pushButton.clicked.connect(lambda: self.Berry_Subtract())
        self.choco_plus_pushButton.clicked.connect(lambda: self.Choco_Add())
        self.choco_minus_pushButton.clicked.connect(lambda: self.Choco_Subtract())
        self.banana_plus_pushButton.clicked.connect(lambda: self.Banana_Add())
        self.banana_minus_pushButton.clicked.connect(lambda: self.Banana_Subtract())
        self.affogato_plus_pushButton.clicked.connect(lambda: self.Affogato_Add())
        self.affogato_voice_pushButton.clicked.connect(lambda: self.Affogato_Voice_Add())
        self.affogato_minus_pushButton.clicked.connect(lambda: self.Affogato_Subtract())

        self.serving_plus_pushButton.clicked.connect(lambda: self.Serving_Add())
        self.serving_minus_pushButton.clicked.connect(lambda: self.Serving_Subtract())
        self.hello_plus_pushButton.clicked.connect(lambda: self.Hello_Add())
        self.hello_minus_pushButton.clicked.connect(lambda: self.Hello_Subtract())
        self.half_plus_pushButton.clicked.connect(lambda: self.Half_Add())
        self.half_minus_pushButton.clicked.connect(lambda: self.Half_Subtract())

        self.order_plus_pushButton.clicked.connect(lambda: self.Order_Add())
        self.order_minus_pushButton.clicked.connect(lambda: self.Order_Subtract())
        self.tableWidget.cellClicked.connect(lambda: self.Cell_Selected())

        self.tableWidget.setColumnWidth(0, 140)
        self.tableWidget.setColumnWidth(1, 140)
        self.tableWidget.setColumnWidth(2, 200)

        self.pay_pushButton.clicked.connect(lambda: self.Sending_Data())

        self.camera = Camera()
        self.camera.start()
        ###################손 제스쳐###################
        self.camera.action.connect(self.handleAction)
        #############################################
    @pyqtSlot(np.ndarray)
    def update_image(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized_frame = cv2.resize(frame, (760, 570))
        h, w, ch = resized_frame.shape
        bytes_per_line = 3 * w
        q_image = QImage(resized_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.webcam_label.setPixmap(QPixmap.fromImage(q_image))


    def recommend_soldout(self, data):
        if data is not None and isinstance(data, list):
            self.berry_recommend_label.hide()
            self.choco_recommend_label.hide()
            self.banana_recommend_label.hide()
            self.affogato_recommend_label.hide()
            self.berry_soldout_label.hide()
            self.choco_soldout_label.hide()
            self.banana_soldout_label.hide()
            self.affogato_soldout_label.hide()
        
        if "strawberry_max" in data:
            self.berry_recommend_label.show()
        if "chocolate_max" in data:
            self.choco_recommend_label.show()
        if "banana_max" in data:
            self.banana_recommend_label.show()
        if "affogato_max" in data:
            self.affogato_recommend_label.show()
        if "strawberry_zero" in data:
            self.berry_recommend_label.hide()
            self.berry_soldout_label.show()
        if "chocolate_zero" in data:
            self.choco_recommend_label.hide()
            self.choco_soldout_label.show()
        if "banana_zero" in data:
            self.banana_recommend_label.hide()
            self.banana_soldout_label.show()
        if "affogato_zero" in data:
            self.affogato_recommend_label.hide()
            self.affogato_soldout_label.show()
    
    def init_ros2(self):
        rclpy.init(args=None)
        self.ros2_client_worker = MinimalClientAsync()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.ros2_client_worker,))
        self.thread.start()

    
    def start_main_page(self, event):
        """메인 페이지로 이동하고 타이머 시작"""
        self.stackedWidget.setCurrentWidget(self.main_page)
        self.camera.detectMode = 1 #나이 성별 판단 모델 동작
        self.ros2_client_worker.send_request2()  # 메뉴별 재고현황 요청
        self.start_main_timer()
    
    def start_main_timer(self):
        """메인 페이지로 이동 시 타이머 시작"""
        # 타이머 설정
        self.timer = QTimer(self)
        self.timer.setInterval(1000)  # 1초마다 타이머 이벤트 발생
        self.remaining_time = 180  # 초기 시간 설정 (180초)
        self.timer.timeout.connect(self.update_timer)
        self.minutes = self.remaining_time // 60
        self.seconds = self.remaining_time % 60
        self.time_Number.display(f"{self.minutes:02d}:{self.seconds:02d}")
        self.timer.start()

        # 타이머 종료 후 페이지 전환
        self.end_timer = QTimer(self)
        self.end_timer.setSingleShot(True)  # 타이머가 한 번만 작동하도록 설정
        self.end_timer.timeout.connect(lambda: self.stackedWidget.setCurrentWidget(self.open_page))
        self.end_timer.start(180000)  # 180초 후 실행

    def update_timer(self):
        self.remaining_time -= 1
        self.minutes = self.remaining_time // 60
        self.seconds = self.remaining_time % 60
        self.time_Number.display(f"{self.minutes:02d}:{self.seconds:02d}")
        if self.remaining_time == 0:
            self.timer.stop()
    
    def check_current_page(self, num):
        if num == 1:
            self.camera.detectMode = 1
            self.category_stackedWidget.setCurrentWidget(self.menu_page)
        elif num == 2:
            self.camera.detectMode = 1
            self.category_stackedWidget.setCurrentWidget(self.option_page)
        elif num == 3:
            self.category_stackedWidget.setCurrentWidget(self.webcam_page)
            self.webcam_label.setPixmap(QPixmap())
            self.camera.detectMode = 2
            self.camera.frame.connect(self.update_image)
        else:
            pass
    
    def update_frame(self):
        global menu, order_num, tracking, shaking, half, order_number, date, price, quantity, gender, age, menu_price
        ret, frame = self.cap.read()
        if ret:
        #     resized_frame = cv2.resize(frame, (760, 570))
            # OpenCV BGR 이미지 -> RGB 이미지 변환
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            height, width, channel = rgb_image.shape
            bytes_per_line = 3 * width
            q_image = QImage(rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888)
            # QImage를 QPixmap으로 변환하여 QLabel에 설정
            pixmap = QPixmap.fromImage(q_image)
            self.webcam_label.setPixmap(pixmap)

            if q_image == "berry" and q_image != "choco" and q_image != "banana":
                menu = "berry"
                self.order_label_1.setText(menu)
                self.order_label_1.setAlignment(Qt.AlignCenter)
                menu_price = 2500
                if q_image == "minus":
                    self.order_label_1.clear()
                    menu = None
                    menu_price = 0

            elif q_image == "choco" and q_image != "berry" and q_image != "banana":
                menu = "choco"
                self.order_label_1.setText(menu)
                self.order_label_1.setAlignment(Qt.AlignCenter)
                menu_price = 2500
                if q_image == "minus":
                    self.order_label_1.clear()
                    menu = None
                    menu_price = 0

            elif q_image == "banana" and q_image != "berry" and q_image != "choco":
                menu = "banana"
                self.order_label_1.setText(menu)
                self.order_label_1.setAlignment(Qt.AlignCenter)
                menu_price = 2000
                if q_image == "minus":
                    self.order_label_1.clear()
                    menu = None
                    menu_price = 0

            elif q_image == "serving":
                tracking = "serving"
                self.order_label_2.setText(tracking)
                self.order_label_2.setAlignment(Qt.AlignCenter)
                if q_image == "minus":
                    self.order_label_2.clear()
                    tracking = None
            
            elif q_image == "hello":
                shaking = "hello"
                self.order_label_3.setText(shaking)
                self.order_label_3.setAlignment(Qt.AlignCenter)
                if q_image == "minus":
                    self.order_label_3.clear()
                    shaking = None

            elif q_image == "half":
                half = "half"
                self.order_label_4.setText(half)
                self.order_label_4.setAlignment(Qt.AlignCenter)
                if q_image == "minus":
                    self.order_label_4.clear()
                    half = None
            
            else:
                pass
    
    def closeEvent(self, event):
        self.camera.stop()
        self.ros2_client_worker.destroy_node()
        rclpy.shutdown()
        event.accept()  # 창을 닫음

    def Berry_Add(self):
        global menu, menu_price
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "choco" and menu != "banana" and menu != "affogato":
            menu = "berry"
            self.order_label_1.setText(menu)
            self.order_label_1.setAlignment(Qt.AlignCenter)
            menu_price = 2500
    
    def Berry_Subtract(self):
        global menu, menu_price

        if menu == "berry":
            self.order_label_1.clear()
            menu = None
            menu_price = 0
    
    def Choco_Add(self):
        global menu, menu_price
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "berry" and menu != "banana" and menu != "affogato":
            menu = "choco"
            self.order_label_1.setText(menu)
            self.order_label_1.setAlignment(Qt.AlignCenter)
            menu_price = 2500
    
    def Choco_Subtract(self):
        global menu, menu_price

        if menu == "choco":
            self.order_label_1.clear()
            menu = None
            menu_price = 0

    def Banana_Add(self):
        global menu, menu_price
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "berry" and menu != "choco" and menu != "affogato":
            menu = "banana"
            self.order_label_1.setText(menu)
            self.order_label_1.setAlignment(Qt.AlignCenter)
            menu_price = 2000
    
    def Banana_Subtract(self):
        global menu, menu_price

        if menu == "banana":
            self.order_label_1.clear()
            menu = None
            menu_price = 0

    def Affogato_Add(self):
        global menu, menu_price
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "berry" and menu != "choco" and menu != "banana":
            menu = "affogato"
            self.order_label_1.setText(menu)
            self.order_label_1.setAlignment(Qt.AlignCenter)
            menu_price = 3000
    
    # def Affogato_Voice_Add(self):
    #     global menu, menu_price
    #     # 글꼴 설정
    #     font = QFont()
    #     font.setPointSize(15)  # 텍스트 크기를 20으로 설정
    #     self.order_label_1.setFont(font)

    #     # 함수 실행
    #     self.recognize_speech_with_apagato() 

    #     server_mac_address = '84:1B:77:04:24:A9'  # 서버 블루투스 MAC 주소
    #     port = 1
    #     server_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    #     server_socket.bind((server_mac_address, port))
    #     server_socket.listen(1)

    #     print("Waiting for a connection...")
    #     client_socket, client_info = server_socket.accept()
    #     print("Accepted connection from ", client_info)

    #     try:
    #         while True:
    #             data = client_socket.recv(1024).decode('utf-8')
    #             if data == "affogato":
    #                 print("Received message: affogato")
    #                 client_socket.send("Message received: affogato")
    #                 menu = "아포가토"
    #                 self.order_label_1.setText(menu)
    #                 self.order_label_1.setAlignment(Qt.AlignCenter)
    #                 menu_price = 3000
    #             else:
    #                 print(f"Received message: {data}")
    #                 client_socket.send("Message received")
    #     except bluetooth.BluetoothError as e:
    #         print(f"Bluetooth Error: {e}")

    #     finally:
    #         client_socket.close()
    #         server_socket.close()
    
    # def recognize_speech(self):
    #     r = sr.Recognizer()
    #     with sr.Microphone() as source:
    #         print("Say Something")
    #         speech = r.listen(source)
    #     try:
    #         audio = r.recognize_google(speech, language="ko-KR")
    #         print("Your speech thinks like\n " + audio)
    #         return audio
    #     except sr.UnknownValueError:
    #         print("Your speech can not understand")
    #         return None
    #     except sr.RequestError as e:
    #         print("Request Error!; {0}".format(e))
    #         return None
        
    # def send_to_server(self, message):
    #     server_mac_address = '84:1B:77:04:24:A9'  # 서버 블루투스 MAC 주소
    #     port = 1
    #     client_socket = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
    #     client_socket.connect((server_mac_address, port))
    #     client_socket.send(message)
    #     client_socket.close()

    # def recognize_speech_with_apagato(self):
    #     audio = self.recognize_speech()
    #     if audio and "affogato" in audio:
    #         print("affogato가 포함된 음성이 인식되었습니다.")
    #         self.send_to_server("affogato")
    #     else:
    #         print("affogato가 포함되지 않은 음성이 인식되었습니다.")
        
    def Affogato_Subtract(self):
        global menu, menu_price

        if menu == "affogato":
            self.order_label_1.clear()
            menu = None
            menu_price = 0
    
    def Serving_Add(self):
        global tracking
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_2.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            tracking = "serving"
            self.order_label_2.setText(tracking)
            self.order_label_2.setAlignment(Qt.AlignCenter)
    
    def Serving_Subtract(self):
        global tracking

        self.order_label_2.clear()
        tracking = None
    
    def Hello_Add(self):
        global shaking
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_3.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            shaking = "hello"
            self.order_label_3.setText(shaking)
            self.order_label_3.setAlignment(Qt.AlignCenter)
    
    def Hello_Subtract(self):
        global shaking

        self.order_label_3.clear()
        shaking = None

    def Half_Add(self):
        global half
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_4.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            half = "half"
            self.order_label_4.setText(half)
            self.order_label_4.setAlignment(Qt.AlignCenter)
    
    def Half_Subtract(self):
        global half

        self.order_label_4.clear()
        half = None

    def Order_Add(self):
        global orders, menu, order_num, tracking, shaking, half, order_number, date, topping, price, quantity, gender, age, menu_price

        if menu != None:
            orders.append({
                'menu': menu,
                'order_num': order_num,
                'tracking': tracking,
                'shaking': shaking,
                'half': half,
                'menu_price': menu_price,
                'topping': topping
            })

            quantity += 1
            price += menu_price

            row = self.tableWidget.rowCount()
            self.tableWidget.insertRow(row)

            menu_item = QTableWidgetItem(menu)
            menu_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 0, menu_item)

            serving_item = QTableWidgetItem(tracking)
            serving_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 1, serving_item)

            hello_item = QTableWidgetItem(shaking)
            hello_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 2, hello_item)

            half_item = QTableWidgetItem(half)
            half_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 3, half_item)

            self.tableWidget.setColumnWidth(0, 140)
            self.tableWidget.setColumnWidth(1, 140)
            self.tableWidget.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
            self.tableWidget.horizontalHeader().setSectionResizeMode(1, QHeaderView.Fixed)
            self.tableWidget.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
            self.tableWidget.horizontalHeader().setSectionResizeMode(3, QHeaderView.Stretch)

            self.quantity_label.setText(str(quantity))
            self.quantity_label.setAlignment(Qt.AlignCenter)
            self.price_label.setText(str(price))
            self.price_label.setAlignment(Qt.AlignCenter)

            self.order_label_1.clear()
            self.order_label_2.clear()
            self.order_label_3.clear()
            self.order_label_4.clear()
            menu = None
            menu_price = 0
            tracking = None
            shaking = None
            half = None
            topping = None
    
    def Order_Subtract(self):
        global orders, menu, order_num, tracking, shaking, half, order_number, date, price, quantity, gender, age, menu_price

        if order_num is not None:
            self.tableWidget.removeRow(order_num)

            quantity -= 1
            price -= orders[order_num]['menu_price']

            if price < 0:
                price = 0
            if quantity < 0:
                quantity = 0

            orders.pop(order_num)

            self.quantity_label.setText(str(quantity))
            self.quantity_label.setAlignment(Qt.AlignCenter)
            self.price_label.setText(str(price))
            self.price_label.setAlignment(Qt.AlignCenter)

            order_num = None

    def Cell_Selected(self):
        global order_num

        order_num = self.tableWidget.currentRow()
    
    def Sending_Data(self):
        global orders, menu, order_num, tracking, shaking, half, order_number, date, topping, price, quantity, gender, age, menu_price

        topping = "topping A"

        order_number += 1 

        current_time = QDateTime.currentDateTime()
        formatted_time = current_time.toString("yyyyMMdd HH:mm")

        for i, order in enumerate(orders):
            # Kiosk - Robot, DB
            menu = str(order['menu'])

            # Kiosk - Robot
            order_num = i + 1
            tracking = str(order['tracking'])
            shaking = str(order['shaking'])
            half = str(order['half'])

            # Kiosk - DB
            order_number
            date = formatted_time
            topping
            price
            quantity
            # gender = "female"  # 딥러닝 적용
            # age = "20~30"  # 딥러닝 적용
            age, gender = self.camera.ageModel.getMostCommonAgeGender()

            self.ros2_client_worker.send_request(order_number, date, menu, topping, price, quantity, gender, age)
            self.ros2_client_worker.send_request2()
            self.ros2_client_worker.send_request3(menu, order_num, tracking, shaking, half)

        orders.clear()
        self.tableWidget.setRowCount(0)
        self.order_label_1.clear()
        self.order_label_2.clear()
        self.order_label_3.clear()
        self.order_label_4.clear()
        self.quantity_label.clear()
        self.price_label.clear()
        menu = None
        menu_price = 0
        tracking = None
        shaking = None
        half = None
        topping = None
        price = 0
        quantity = 0

    @pyqtSlot(str)
    def handleAction(self, action):
        print(action)
        
class Camera(QObject):
    frame = pyqtSignal(np.ndarray)
    action = pyqtSignal(str)
    
    def __init__(self, camera_index=0):
        super().__init__()
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise Exception(f"Camera with index {self.camera_index} could not be opened.")
        
        self.gestureModel = GestureModel()
        self.ageModel = AgeModel()
        self.running = False
        self.thread = None
        self.detectMode = 0 #0 : 얼굴만 인식, 1: 나이 성별 인식, 2: 제스처 인식
        self.gestureDetected = False

        if torch.cuda.is_available():
            # GPU를 사용하도록 설정
            self.device = torch.device("cuda")
            print("GPU를 사용합니다.")
        else:
            # CPU를 사용하도록 설정
            self.device = torch.device("cpu")
            print("GPU를 사용할 수 없습니다. CPU를 사용합니다.")

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            if self.detectMode == 0:
                frame, _, _ = self.ageModel.detectFace(frame)
            elif self.detectMode == 1:
                frame = self.ageModel.analyzeAgeGender(frame)
            elif self.detectMode == 2:
                frame, action = self.gestureModel.analyzeGesture(frame)
                self.frame.emit(frame)
                if action != '?':
                    if self.gestureDetected == False:
                        self.action.emit(str(action))
                        self.gestureDetected = True
                else:
                        self.gestureDetected = False

    def stop(self):
        self.running = False
        if self.thread is not None:
            self.thread.join()
        self.cap.release()

def main():
    app = QApplication(sys.argv)
    myWindow = FirstClass()
    myWindow.show()
    app.exec_()

if __name__ == '__main__':
    main()
