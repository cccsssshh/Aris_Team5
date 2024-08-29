import sys
import os
from ament_index_python.packages import get_package_share_directory

current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
package_share_directory = get_package_share_directory('kiosk_package')
ui_file_path = os.path.join(package_share_directory, 'ui/kiosk.ui')

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
from gtts import gTTS


first_class = uic.loadUiType(ui_file_path)[0]

"""UI(FirstClass)의 여러 메서드에서 사용하는 변수를 전역변수로 선언"""
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

class Kiosk(Node, QObject):
    """Robot과 DB로 request, response를 수행하는 클라이언트 노드를 클래스로 선언"""

    result_ready = pyqtSignal(int)
    restquantity = pyqtSignal(object)

    def __init__(self):
        """클라이언트 객체 생성"""

        super().__init__('Kiosk')
        QObject.__init__(self)
        self.cli = self.create_client(OrderRecord, 'OrderRecord')
        self.cli2 = self.create_client(RestQuantity, 'RestQuantity')
        self.cli3 = self.create_client(IceRobot, "IceRobot")
        self.get_logger().info('node start')
        
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        # while not self.cli2.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        
        while not self.cli3.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = OrderRecord.Request()
        self.req2 = RestQuantity.Request()
        self.req3 = IceRobot.Request()

    @pyqtSlot(int, str, str, str, int, int, str, str)
    def send_request(self, order_number, date, menu, topping, price, quantity, gender, age):
        """DB로 주문 데이터 request하고 callback 함수 호출"""

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
        """DB의 response를 받아서 터미널 창에 출력"""

        try:
            response = future.result()
            self.get_logger().info(f'DB_Result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def send_request2(self):
        """DB에서 각 메뉴 재고를 받아오도록 request신호만 보냄"""

        self.get_logger().info("connected")
        self.future2 = self.cli2.call_async(self.req2)
        self.future2.add_done_callback(self.future2_callback)
    
    def future2_callback(self, future2):
        """DB로부터 재고를 받아오면 각 재고량을 비교해서 restquantity시그널에 emit함"""

        try:
            response = future2.result()
            self.get_logger().info(f'berry: {response.strawberry}, choco: {response.chocolate}, banana: {response.banana}, affogato: {response.affogato}')
            
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
    def send_request3(self, menu, order_num, shaking, half, tracking):
        """Robot 노드로 주문 데이터를 request함"""

        self.req3.menu = menu
        self.req3.order_num = order_num
        self.req3.shaking = shaking
        self.req3.half = half
        self.req3.tracking = tracking
        self.future3 = self.cli3.call_async(self.req3)
        self.future3.add_done_callback(self.future3_callback)

    def future3_callback(self, future3):
        """Robot 노드로부터 response를 받아서 터미널 창에 출력"""
        try:
            response = future3.result()
            self.get_logger().info(f'Robot_Result: {response.success}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

class SpeechRecognitionThread(QObject):
    """UI 프로그램과는 별도로 음성인식을 수행하기 위한 클래스"""

    voice_recognition = pyqtSignal(str)

    def __init__(self):
        """초기 설정"""

        super().__init__()
        self.thread = None

    def start(self):
        """음성인식 메서드를 스레드 객체로 생성 후 실행함"""

        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.voice_run)
            self.thread.start()

    def voice_run(self):
        """구글 API를 이용해서 음성을 한글 텍스트로 변환한 후 voice_recognition 시그널에 emit함"""

        r = sr.Recognizer()
        with sr.Microphone() as source:
            print("Say Something")
            speech = r.listen(source, timeout=5, phrase_time_limit=10)
        try:
            audio = r.recognize_google(speech, language="ko-KR")
            print("Your speech thinks like\n " + audio)
            self.voice_recognition.emit(audio)
        except sr.UnknownValueError:
            print("Your speech can not understand")
            self.voice_recognition.emit("")
        except sr.RequestError as e:
            print("Request Error!; {0}".format(e))
            self.voice_recognition.emit("")
        finally:
            self.thread = None 
    
    def voice_stop(self):
        """음성인식 스레드를 정지시킴"""

        if self.thread is not None:
            self.thread.join()
            self.thread = None


class FirstClass(QMainWindow,first_class):
    """UI 프로그램 클래스"""

    clicked = pyqtSignal()

    def __init__(self):
        """초기 설정"""

        super().__init__()
        self.setupUi(self)
        self.setWindowTitle("Main Page")

        self.client_node_thread()
        self.camera_class_call()

        self.ros2_client_worker.get_logger().info('ros2 client worker start')

        # 오픈화면 #######################################################################################################
        self.stackedWidget.setCurrentIndex(0)  # 시작할때 화면은 오픈 페이지로 설정
        self.set_ad_image()  
        self.setWindowFlags(Qt.FramelessWindowHint) # 프레임 지우기
        self.move(10,30) #창이동

        # 페이지 이동 및 타이머 시작
        self.ad_label.mousePressEvent = self.start_main_page # 페이지 이동

        # 메인화면 #######################################################################################################
        self.berry_label.setPixmap(QPixmap('/home/k/Documents/QT/berry.webp').scaled(QSize(377, 198)))
        self.choco_label.setPixmap(QPixmap('/home/k/Documents/QT/choco.webp').scaled(QSize(377, 198)))
        self.banana_label.setPixmap(QPixmap('/home/k/Documents/QT/banana.webp').scaled(QSize(377, 198)))
        self.affogato_label.setPixmap(QPixmap('/home/k/Documents/QT/affogato.jpg').scaled(QSize(377, 198)))
        self.serving_label.setPixmap(QPixmap('/home/k/Documents/QT/serving.jpg').scaled(QSize(243, 214)))
        self.hello_label.setPixmap(QPixmap('/home/k/Documents/QT/hand.jpg').scaled(QSize(243, 214)))
        self.half_label.setPixmap(QPixmap('/home/k/Documents/QT/half_half.jpg').scaled(QSize(243, 214)))

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

        self.order_status = 0
        self.check_flag = False

        self.second_menu_add = False


    
    ### 별도의 스레드 관리 (클라이언트 노드, 카메라) ############################################################################

    def client_node_thread(self):
        """client node 클래스를 별도의 스레드로 생성,실행"""

        rclpy.init(args=None)
        self.ros2_client_worker = Kiosk()
        self.thread = threading.Thread(target=rclpy.spin, args=(self.ros2_client_worker,))
        self.thread.start()
    
    def camera_class_call(self):
        """Camera 클래스 호출"""

        self.camera = Camera()
        self.camera.start()


    
    ### 오픈화면 -> 메인화면 ###############################################################################################
        
    def set_ad_image(self):
        """오픈화면에 적용되는 광고 이미지를 불러옴"""

        self.ad_label.setPixmap(QPixmap('/home/k/ros2_ws_local/src/kiosk_package/ui/image/ARIS.jpg').scaled(QSize(768, 1024)))
    
    def add_page_mouse_press(self, event):
        """오픈화면 누르면 메인화면으로 이동"""

        self.stackedWidget.setCurrentWidget(self.main_page)



    ### 메인화면 이동 후 동작 설정 ##########################################################################################
    
    def start_main_page(self, event):
        """메인 페이지로 이동하고 성별/나이 모델 동작, 재고현황 요청, 3분 타이머 호출"""

        self.category_stackedWidget.setCurrentWidget(self.menu_page)
        self.camera.detectMode = 1 #나이 성별 판단 모델 동작
        self.ros2_client_worker.send_request2()  # 메뉴별 재고현황 요청

        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()
        if hasattr(self, 'end_timer') and self.end_timer.isActive():
            self.end_timer.stop()

        self.start_main_timer()
    
    def start_main_timer(self):
        """타이머 시작, 3분 후 오픈화면으로 전환"""

        if hasattr(self, 'timer') and self.timer.isActive():
            self.timer.stop()

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
        if hasattr(self, 'end_timer') and self.end_timer.isActive():
            self.end_timer.stop()

        self.end_timer = QTimer(self)
        self.end_timer.setSingleShot(True)  # 타이머가 한 번만 작동하도록 설정
        self.end_timer.timeout.connect(lambda: self.stackedWidget.setCurrentWidget(self.open_page))
        self.end_timer.start(180000)  # 180초 후 실행

    def update_timer(self):
        """3분 카운트다운을 실시간으로 time_Number 라벨에 표시"""

        self.remaining_time -= 1
        self.minutes = self.remaining_time // 60
        self.seconds = self.remaining_time % 60
        self.time_Number.display(f"{self.minutes:02d}:{self.seconds:02d}")
        if self.remaining_time == 0:
            self.timer.stop()



    ### MENU, OPTION, 장애인용 버튼 입력에 따른 창 전환 제어 #################################################################
    
    def check_current_page(self, num):
        """버튼 입력에 따라 창 전환, 카메라 모드 전환 및 TTS 안내"""

        if num == 1:
            self.camera.detectMode = 1
            self.category_stackedWidget.setCurrentWidget(self.menu_page)
        elif num == 2:
            self.camera.detectMode = 1
            self.category_stackedWidget.setCurrentWidget(self.option_page)
        elif num == 3:
            self.category_stackedWidget.setCurrentWidget(self.webcam_page)
            self.camera.detectMode = 2
            self.camera.frame.connect(self.update_image)
            self.camera.action.connect(self.handleAction)

            text = "시각장애인용 키오스크입니다. 딸기맛은 하나, 초코맛은 둘, 바나나맛은 셋으로 손동작을 보여주세요."
            tts = gTTS(text=text, lang='ko')
            tts.save("output.mp3")
            # os.system("mpg321 output.mp3")
            print("시각장애인용 키오스크입니다. 딸기맛은 하나, 초코맛은 둘, 바나나맛은 셋으로 손동작을 보여주세요.")
        else:
            pass
    
    @pyqtSlot(np.ndarray)
    def update_image(self, frame):
        """UI의 webcam_label 크기에 맞춰서 웹캠 영상 출력"""
        
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        resized_frame = cv2.resize(frame, (760, 570))
        h, w, ch = resized_frame.shape
        bytes_per_line = 3 * w
        q_image = QImage(resized_frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
        self.webcam_label.setPixmap(QPixmap.fromImage(q_image))
    
    
    ### 버튼 방식 주문 제어 ##################################################################################################

    def Berry_Add(self):
        global menu, menu_price
        # 글꼴 설정
        font = QFont()
        font.setPointSize(15)  # 텍스트 크기를 20으로 설정
        self.order_label_1.setFont(font)

        if self.berry_plus_pushButton.isEnabled() and menu != "choco" and menu != "banana" and menu != "affogato" and menu != "아포가토":
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

        if self.choco_plus_pushButton.isEnabled() and menu != "berry" and menu != "banana" and menu != "affogato" and menu != "아포가토":
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

        if self.banana_plus_pushButton.isEnabled() and menu != "berry" and menu != "choco" and menu != "affogato" and menu != "아포가토":
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

        if self.affogato_plus_pushButton.isEnabled() and menu != "berry" and menu != "choco" and menu != "banana" and menu != "아포가토":
            menu = "affogato"
            self.order_label_1.setText(menu)
            self.order_label_1.setAlignment(Qt.AlignCenter)
            menu_price = 3000
    
    def Affogato_Voice_Add(self):
        global menu, menu_price
        
        self.voice_thread = SpeechRecognitionThread()
        self.voice_thread.start()
        self.voice_thread.voice_recognition.connect(self.handle_speech_result)

    def handle_speech_result(self, audio):
        global menu, menu_price

        if audio and "아포가토" in audio:
            print("아포가토가 포함된 음성이 인식되었습니다.")
            if self.affogato_plus_pushButton.isEnabled() and menu != "berry" and menu != "choco" and menu != "banana" and menu != "affogato":
                # 글꼴 설정
                font = QFont()
                font.setPointSize(15)  # 텍스트 크기를 20으로 설정
                self.order_label_1.setFont(font)

                menu = "아포가토"
                self.order_label_1.setText(menu)
                self.order_label_1.setAlignment(Qt.AlignCenter)
                menu_price = 3000
        else:
            print("아포가토가 포함되지 않은 음성이 인식되었습니다.")
        
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

        if self.serving_plus_pushButton.isEnabled():
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

        if self.hello_plus_pushButton.isEnabled():
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

        if self.half_plus_pushButton.isEnabled():
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

            # self.ros2_client_worker.send_request(order_number, date, menu, topping, price, quantity, gender, age)
            # self.ros2_client_worker.send_request2()
            self.ros2_client_worker.send_request3(menu, order_num, shaking, half, tracking)

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
        order_num = 0
        tracking = None
        shaking = None
        half = None
        topping = None
        price = 0
        quantity = 0

        # self.stackedWidget.setCurrentWidget(self.open_page)

    
    ### 장애인용 손동작 주문 기능 제어 #######################################################################################
    
    @pyqtSlot(str)
    def handleAction(self, action):
        """TTS 안내로 주문 순차 진행, action 값에 따라 지정한 문자열을 주문내역 라벨에 표시 및 주문"""

        global orders, menu, order_num, tracking, shaking, half, order_number, date, topping, price, quantity, gender, age, menu_price

        print(action)

        # 변환할 텍스트 목록을 정의합니다
        # texts = [
        #     "딸기맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요.", # output_1.mp3
        #     "초코맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요.", # output_2.mp3
        #     "바나나맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요.", # output_3.mp3
        #     "메뉴가 추가되었습니다", # output_4.mp3
        #     "다른 맛을 선택해주십시요.", # output_5.mp3
        #     "서빙 옵션을 추가하려면 o, 아니라면 x로 손동작을 보여주세요.", # output_6.mp3
        #     "인사 옵션을 추가하려면 o, 아니라면 x로 손동작을 보여주세요.", # output_7.mp3
        #     "반반 옵션을 추가하려면 o, 아니라면 x로 손동작을 보여주세요.", # output_8.mp3
        #     "추가되었습니다", # output_9.mp3
        #     "다음 옵션으로 넘어갑니다.", # output_10.mp3
        #     "주문하시겠습니까? 맞다면 o, 아니라면 x표로 손동작을 보여주세요.", # output_11.mp3
        #     "주문되었습니다.", # output_12.mp3
        #     "주문이 취소되었습니다."  # output_13.mp3
        # ]

        # # 각 텍스트를 변환하여 별도의 파일로 저장합니다
        # for i, text in enumerate(texts):
        #     tts = gTTS(text=text, lang='ko')
        #     filename = f"output_{i+1}.mp3"
        #     tts.save(filename)
        
        if self.order_status == 0:  # 메뉴 선택
            if not self.check_flag: 
                menu = self.menu_selected(action)
                menu_price = self.price_selected(action)
                self.check_flag = True
            else:
                check = self.checking(action)
                if check:
                    font = QFont()
                    font.setPointSize(15)  
                    self.order_label_1.setFont(font)

                    self.order_label_1.setText(menu)
                    self.order_label_1.setAlignment(Qt.AlignCenter)

                    self.order_status += 1
                    self.check_flag = False
                    print("서빙 옵션을 추가하려면 o, 아니라면 x로 손동작을 보여주세요.")
                else:
                    print("다시 시도하세요")
        elif self.order_status == 1:  # 서빙여부 선택
            check = self.checking(action)
            if check:
                font = QFont()
                font.setPointSize(15)  
                self.order_label_2.setFont(font)

                tracking = "serving"
                self.order_label_2.setText(tracking)
                self.order_label_2.setAlignment(Qt.AlignCenter)

                self.order_status += 1
                print("인사 옵션을 추가하려면 o, 아니라면 x로 손동작을 보여주세요.")
            else:
                print("다시 시도하세요")
        elif self.order_status == 2:  # 인사여부 선택
            check = self.checking(action)
            if check:
                font = QFont()
                font.setPointSize(15)  
                self.order_label_3.setFont(font)

                shaking = "hello"
                self.order_label_3.setText(shaking)
                self.order_label_3.setAlignment(Qt.AlignCenter)

                self.order_status += 1
                print("반반 옵션을 추가하려면 o, 아니라면 x로 손동작을 보여주세요.")
            else:
                print("다시 시도하세요")
        elif self.order_status == 3:  # 반반여부 선택
            check = self.checking(action)
            if check:
                font = QFont()
                font.setPointSize(15)  
                self.order_label_4.setFont(font)

                half = "half"
                self.order_label_4.setText(half)
                self.order_label_4.setAlignment(Qt.AlignCenter)

                topping = "topping A"
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

                self.order_status += 1
                print("주문하시겠습니까? 맞다면 o, 아니라면 x표로 손동작을 보여주세요.")
            else:
                print("다시 시도하세요")
        elif self.order_status == 4:  # 주문여부 선택
            check = self.checking(action)
            if check:
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

                    # self.ros2_client_worker.send_request(order_number, date, menu, topping, price, quantity, gender, age)
                    # self.ros2_client_worker.send_request2()
                    # self.ros2_client_worker.send_request3(menu, order_num, shaking, half, tracking)

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
                order_num = 0
                tracking = None
                shaking = None
                half = None
                topping = None
                price = 0
                quantity = 0

                self.order_status = 0
                self.check_flag = False

                # self.stackedWidget.setCurrentWidget(self.open_page)
            else:
                print("다시 시도하세요")
    
    def menu_selected(self, action):
        """action 1, 2, 3 값에 따라 menu 변수에 지정한 문자열 대입"""

        global menu

        if action == "1":
            # os.system("mpg321 output_1.mp3")  # "딸기맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요."
            print("딸기맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요.")
            menu = "berry"
        
        elif action == "2":
            # os.system("mpg321 output_1.mp3")  # "딸기맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요."
            print("초코맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요.")
            menu = "choco"
        
        elif action == "3":
            # os.system("mpg321 output_1.mp3")  # "딸기맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요."
            print("바나나맛을 선택하셨습니다. 맞다면 o, 아니라면 x로 손동작을 보여주세요.")
            menu = "banana"
            
        return menu

    def price_selected(self, action):
        """action 1, 2, 3 값에 따라 menu_price에 지정한 값 대입"""

        global menu_price

        if action == "1":
            menu_price = 2500
        
        elif action == "2":
            menu_price = 2500
        
        elif action == "3":
            menu_price = 2000
        
        return menu_price
    
    def checking(self, action):
        """action O, X 값에 따라 필요한 TTS 안내"""

        if action == "O":
            if self.order_status != 4:
                print("추가되었습니다.")
                return True
            elif self.order_status == 4:
                print("주문되었습니다.")
                return True
        elif action == "X":
            print("취소되었습니다.")
            return False
        

    ### 재고량에 따른 추천, 품절 표시 기능 제어 ####################################################################################################

    def recommend_soldout(self, data):
        """restquantity 신호에 따라 추천, 품절 label 보여줌"""

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
    

    ### UI 종료 시 카메라, 클라이언트 노드 정지 제어 ##################################################################################
    
    def closeEvent(self, event):
        """카메라 정지, 클라이언트 노드 파괴"""

        self.camera.stop()
        self.ros2_client_worker.destroy_node()
        rclpy.shutdown()
        event.accept()  # 창을 닫음
        
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
