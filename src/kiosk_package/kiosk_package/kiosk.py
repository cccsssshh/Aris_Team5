from interface_package.srv import IceRobot
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
import uuid
import cv2
import numpy as np
import mediapipe as mp
from tensorflow.keras.models import load_model

first_class = uic.loadUiType("/home/k/ros2_ws_git/Aris_Team5/src/kiosk_package/ui/kiosk.ui")[0]

# Kiosk - Robot, DB
menu = None  # 메뉴

# Kiosk - Robot
order_num = None  # 주문 개수
shaking = None  # handling
tracking = None  # serving
gesture = None  # gesture order
half = None  # 반반 주문
data = None

# Kiosk - DB
order_number = 0  # 주문번호
date = None  # 주문날짜, 시간
price = 0  # 총 금액
quantity = 0  # 총 수량
gender = None  # 성별
age = None  # 연령대

menu_price = 0  # 주문 시에 쓰는 임시 변수
orders = []  # 주문 내역 임시 저장 변수 (리스트)

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli= self.create_client(IceRobot, 'IceRobot')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        self.cap = cv2.VideoCapture(0)
        self.this_action = '?' # 현재 동작
        self.actions = ['banana', 'choco', 'berry'] # 가능한 동작 리스트
        self.seq_length = 30 # 시퀀스 길이
        self.model = load_model('/home/k/ros2_ws/src/robot_package/recog_model/model2_1.0.keras') # 동작 인식 모델
        self.seq = [] # 관절 데이터 시퀀스
        self.action_seq = [] # 동작 시퀀스
        self.mp_hands = mp.solutions.hands # 미디어파이프 손 모듈
        self.mp_drawing = mp.solutions.drawing_utils # 미디어파이프 그리기 도구
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8
        )

        self.req = IceRobot.Request()

    def send_request(self, menu, order_num, shaking, tracking, gesture, half, data):
        self.req.menu = menu
        self.req.order_num = order_num
        self.req.shaking = shaking
        self.req.tracking = tracking
        self.req.gesture = gesture
        self.req.half = half  
        self.data = data                               
        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_request2(self, action):
        req = IceRobot.Request()
        req.data = action
        # future2 = self.client.call_async(req2)
        # rclpy.spin_until_future_complete(self, future2)
        # return future2.result()
    
    def final_hand_machine(self):
        while self.cap.isOpened():
            ret, img = self.cap.read()
            if not ret:
                break
            img = cv2.flip(img, 1)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            result = self.hands.process(img)
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if result.multi_hand_landmarks is not None:
                for res in result.multi_hand_landmarks:
                    joint = np.zeros((21, 4))
                    for j, lm in enumerate(res.landmark):
                        joint[j] = [lm.x, lm.y, lm.z, lm.visibility]
                    v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3]
                    v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3]
                    v = v2 - v1
                    v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]
                    angle = np.arccos(np.einsum('nt,nt->n',
                        v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:],
                        v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:]))
                    angle = np.degrees(angle)
                    d = np.concatenate([joint.flatten(), angle])
                    self.seq.append(d)
                    self.mp_drawing.draw_landmarks(img, res, self.mp_hands.HAND_CONNECTIONS)
                    if len(self.seq) < self.seq_length:
                        continue
                    input_data = np.expand_dims(np.array(self.seq[-self.seq_length:], dtype=np.float32), axis=0)
                    y_pred = self.model.predict(input_data).squeeze()
                    i_pred = int(np.argmax(y_pred))
                    conf = y_pred[i_pred]
                    if conf < 0.93:
                        continue
                    self.action = self.actions[i_pred]
                    self.action_seq.append(self.action)
                    if len(self.action_seq) < 10:
                        continue
                    if self.action_seq[-1] == self.action_seq[-2] == self.action_seq[-3] == self.action_seq[-4] == self.action_seq[-5] == self.action_seq[-6] == self.action_seq[-7] == self.action_seq[-8] == self.action_seq[-9] == self.action_seq[-10]:
                        self.this_action = self.action
                    cv2.putText(img, f'{self.this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)
            if self.this_action != '?':
                self.send_request2(self.this_action)
                # if response.success:
                #     self.speak(f"{self.this_action} 아이스크림 주문 받았습니다.")
                #     # Add additional logic for audio and robot control if necessary
                self.this_action = '?'
                self.action_seq.clear()
                self.seq.clear()
            cv2.imshow('img', img)
            if cv2.waitKey(1) == ord('q'):
                break
            
        self.cap.release()
        cv2.destroyAllWindows()

# PyQt5와 ROS 2 통합을 위한 워커 객체를 정의합니다
class ROS2ClientWorker(QObject):
    # 결과를 전달하기 위한 시그널을 정의합니다
    result_ready = pyqtSignal(int)

    def __init__(self):
        super().__init__()
        self.node = None
        self.client = None
        self.thread = threading.Thread(target=self.init_ros2)
        self.thread.start()
    def init_ros2(self):
        rclpy.init(args=None)
        self.node = MinimalClientAsync()
        rclpy.spin(self.node)
    
    @pyqtSlot(str, int, str, str, str, str)
    def send_request(self, menu, order_num, shaking, tracking, gesture, half, data):
        if self.node is not None:
            self.node.send_request(menu, order_num, shaking, tracking, gesture, half, data)

class FirstClass(QMainWindow,first_class):
    """오픈화면 & 메인화면 창"""
    clicked = pyqtSignal()

    def add_page_mouse_press(self, event):
        """오픈화면 누르면 발생하는 이벤트"""
        self.stackedWidget.setCurrentWidget(self.main_page)
    
    # 이미지 처리 함수 ####################################################################################################
    def set_ad_image(self):
        self.ad_label.setPixmap(QPixmap('/home/k/Documents/QT/ARIS.jpg').scaled(QSize(768, 1024)))  # 첫번째 이미지

    ####################################################################################################################

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main Page")
        self.ros2_client_worker = ROS2ClientWorker()

        # 오픈화면 #######################################################################################################
        self.stackedWidget.setCurrentIndex(0)  # 시작할때 화면은 오픈 페이지로 설정
        self.set_ad_image()  
        self.setWindowFlags(Qt.FramelessWindowHint) # 프레임 지우기
        self.move(10,30) #창이동

        # 페이지 이동 및 타이머 시작
        self.ad_label.mousePressEvent = self.start_main_page # 페이지 이동)

        # 메인화면 #######################################################################################################
        self.berry_label.setPixmap(QPixmap('/home/k/Documents/QT/berry.webp').scaled(QSize(377, 198)))
        self.choco_label.setPixmap(QPixmap('/home/k/Documents/QT/choco.webp').scaled(QSize(377, 198)))
        self.banana_label.setPixmap(QPixmap('/home/k/Documents/QT/banana.webp').scaled(QSize(377, 198)))
        self.affogato_label.setPixmap(QPixmap('/home/k/Documents/QT/affogato.jpg').scaled(QSize(377, 198)))
        self.serving_label.setPixmap(QPixmap('/home/k/Documents/QT/serving.jpg').scaled(QSize(243, 247)))
        self.hand_label.setPixmap(QPixmap('/home/k/Documents/QT/hand.jpg').scaled(QSize(243, 247)))
        # self.half_label.setPixmap(QPixmap('/home/k/Documents/QT/camera.jpg').scaled(QSize(243, 247)))

        self.button_pushButton.clicked.connect(lambda: self.check_current_page(1))
        self.gesture_pushButton.clicked.connect(lambda: self.check_current_page(2))
        self.option_pushButton.clicked.connect(lambda: self.check_current_page(3))

        self.berry_plus_pushButton.clicked.connect(lambda: self.Berry_Add())
        self.berry_minus_pushButton.clicked.connect(lambda: self.Berry_Subtract())
        self.choco_plus_pushButton.clicked.connect(lambda: self.Choco_Add())
        self.choco_minus_pushButton.clicked.connect(lambda: self.Choco_Subtract())
        self.banana_plus_pushButton.clicked.connect(lambda: self.Banana_Add())
        self.banana_minus_pushButton.clicked.connect(lambda: self.Banana_Subtract())
        self.affogato_plus_pushButton.clicked.connect(lambda: self.Affogato_Add())
        self.affogato_voice_pushButton.clicked.connect(lambda: self.Affogato_Voice_Add())
        self.affogato_minus_pushButton.clicked.connect(lambda: self.Affogato_Subtract())

        self.gesture_plus_pushButton.clicked.connect(lambda: self.Gesture_Add())
        self.gesture_minus_pushButton.clicked.connect(lambda: self.Gesture_Subtract())

        self.serving_plus_pushButton.clicked.connect(lambda: self.Serving_Add())
        self.serving_minus_pushButton.clicked.connect(lambda: self.Serving_Subtract())
        self.hand_plus_pushButton.clicked.connect(lambda: self.Hand_Add())
        self.hand_minus_pushButton.clicked.connect(lambda: self.Hand_Subtract())
        self.half_plus_pushButton.clicked.connect(lambda: self.Half_Add())
        self.half_minus_pushButton.clicked.connect(lambda: self.Half_Subtract())

        self.order_plus_pushButton.clicked.connect(lambda: self.Order_Add())
        self.order_minus_pushButton.clicked.connect(lambda: self.Order_Subtract())
        self.tableWidget.cellClicked.connect(lambda: self.Cell_Selected())

        self.tableWidget.setColumnWidth(0, 140)
        self.tableWidget.setColumnWidth(1, 140)
        self.tableWidget.setColumnWidth(2, 200)

        self.pay_pushButton.clicked.connect(lambda: self.Sending_Data())
    
    def start_main_page(self, event):
        """메인 페이지로 이동하고 타이머 시작"""
        self.stackedWidget.setCurrentWidget(self.main_page)
        self.start_main_timer()
    
    def start_main_timer(self):
        global orders, menu, order_num, shaking, tracking, gesture, half, order_number, date, price, quantity, gender, age, menu_price

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
            self.category_stackedWidget.setCurrentWidget(self.menu_page)
        elif num == 2:
            self.category_stackedWidget.setCurrentWidget(self.topping_page)
        elif num == 3:
            self.category_stackedWidget.setCurrentWidget(self.option_page)
        else:
            pass
    
    def Berry_Add(self):
        global menu, menu_price, gesture
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "choco" and menu != "banana" and menu != "affogato" and gesture != "gesture":
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
        global menu, menu_price, gesture
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "berry" and menu != "banana" and menu != "affogato" and gesture != "gesture":
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
        global menu, menu_price, gesture
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
        global menu, menu_price, gesture
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
    #     global menu, menu_price, gesture
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

    def Gesture_Add(self):
        global gesture
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_2.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            gesture = "gesture"
            self.order_label_2.setText(gesture)
            self.order_label_2.setAlignment(Qt.AlignCenter)

    def Gesture_Subtract(self):
        global gesture

        if gesture == "gesture":
            self.order_label_2.clear()
            gesture = None
    
    def Serving_Add(self):
        global tracking
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_3.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            tracking = "serving"
            self.order_label_3.setText(tracking)
            self.order_label_3.setAlignment(Qt.AlignCenter)
    
    def Serving_Subtract(self):
        global tracking

        self.order_label_3.clear()
        tracking = None
    
    def Hand_Add(self):
        global shaking
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_4.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            shaking = "hello"
            self.order_label_4.setText(shaking)
            self.order_label_4.setAlignment(Qt.AlignCenter)
    
    def Hand_Subtract(self):
        global shaking

        self.order_label_4.clear()
        shaking = None

    def Half_Add(self):
        global half
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_5.setFont(font)

        if self.berry_plus_pushButton.isEnabled():
            half = "half"
            self.order_label_5.setText(half)
            self.order_label_5.setAlignment(Qt.AlignCenter)
    
    def Half_Subtract(self):
        global half

        self.order_label_5.clear()
        half = None

    def Order_Add(self):
        global orders, menu, order_num, shaking, tracking, gesture, half, order_number, date, price, quantity, gender, age, menu_price

        if menu != None:
            orders.append({
                'menu': menu,
                'order_num': order_num,
                'shaking': shaking,
                'tracking': tracking,
                'gesture': gesture,
                'half': half,
                'menu_price': menu_price
            })

            quantity += 1
            price += menu_price

            combined_text = f"{shaking} / {tracking} / {half}"
            row = self.tableWidget.rowCount()
            self.tableWidget.insertRow(row)

            menu_item = QTableWidgetItem(menu)
            menu_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 0, menu_item)

            gesture_item = QTableWidgetItem(f"{gesture}")
            gesture_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 1, gesture_item)

            combined_item = QTableWidgetItem(combined_text)
            combined_item.setTextAlignment(Qt.AlignCenter)
            self.tableWidget.setItem(row, 2, combined_item)

            self.tableWidget.setColumnWidth(0, 140)
            self.tableWidget.setColumnWidth(1, 140)
            self.tableWidget.horizontalHeader().setSectionResizeMode(0, QHeaderView.Fixed)
            self.tableWidget.horizontalHeader().setSectionResizeMode(1, QHeaderView.Fixed)
            self.tableWidget.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)

            self.quantity_label.setText(str(quantity))
            self.quantity_label.setAlignment(Qt.AlignCenter)
            self.price_label.setText(str(price))
            self.price_label.setAlignment(Qt.AlignCenter)

            self.order_label_1.clear()
            self.order_label_2.clear()
            self.order_label_3.clear()
            self.order_label_4.clear()
            self.order_label_5.clear()
            menu = None
            menu_price = 0
            shaking = None
            tracking = None
            gesture = None
            half = None
    
    def Order_Subtract(self):
        global orders, menu, order_num, shaking, tracking, gesture, half, order_number, date, price, quantity, gender, age, menu_price

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
        global orders, menu, order_num, shaking, tracking, gesture, half, order_number, date, price, quantity, gender, age, menu_price

        order_number += 1 

        current_time = QDateTime.currentDateTime()
        formatted_time = current_time.toString("yyyyMMdd HH:mm")

        for i, order in enumerate(orders):
            # Kiosk - Robot, DB
            menu = str(order['menu'])

            # Kiosk - Robot
            order_num = i + 1
            shaking = str(order['shaking'])
            tracking = str(order['tracking'])
            gesture = str(order['gesture'])
            half = str(order['half'])

            # Kiosk - DB
            order_number
            date = formatted_time
            price
            quantity
            gender = "female"  # 딥러닝 적용
            age = "20~30"  # 딥러닝 적용

            self.ros2_client_worker.send_request(menu, order_num, shaking, tracking, gesture, half, data)

        orders.clear()
        self.tableWidget.setRowCount(0)
        self.order_label_1.clear()
        self.order_label_2.clear()
        self.order_label_3.clear()
        self.order_label_4.clear()
        self.order_label_5.clear()
        self.price_label.clear()
        self.quantity_label.clear()
        menu = None
        menu_price = 0
        shaking = None
        tracking = None
        gesture = None
        half = None
        price = 0
        quantity = 0

def main(args=None):
    app = QApplication(sys.argv)
    myWindow = FirstClass()
    myWindow.show()
    app.exec_()

if __name__ == '__main__':
    main()
