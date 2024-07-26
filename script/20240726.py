
#!/usr/bin/env python3

# S/N : XYZARIS0V3P2311N02
# Robot IP : 192.168.1.192
# code_version : 3.1.5.2

# ============================ library, module ================================ #
import cv2
import mediapipe as mp
import numpy as np
import socket
import json
import os
import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
import bluetooth
import speech_recognition as sr
import playsound
import pygame

from threading import Timer 
from scipy.optimize import minimize
from xarm import version
from xarm.wrapper import XArmAPI
from tensorflow.keras.models import load_model
from threading import Thread, Event
from playsound import playsound
from gtts import gTTS


# ============================= ARIS ROBOT ARM SYSTEM =============================== #
class RobotMain(object):
    """Robot Main Class"""
    
    # =============================== init parameter ========================================= #    
    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        self.state = 'stopped'
        self.start = 0
        self.audio_file = 'refresh.mp3'
        self.game_file = 'MPMP.mp3'
        self.current_state = np.array([269.4, -20.1, 20.4, 176.9, 53.4, -1.1])
        self.dh_params = [ [0, 243.3, 0, -90],
                           [-90, 0, 200, 180],
                           [-90, 0, 87, 90],
                           [0, 227.6, 0, 90],
                           [0, 0, 0, -90],
                           [0, 61.5, 0, 0] ]
        self.coordinates = []
        self.solutions = []
        self.last_processed_time = time.time()
        self.process_interval = 0.75  # 1초 간격으로 좌표 처리
        self.cleanup_interval = 3 # per 3 sec
        self.stop_event = threading.Event()
        self.cleanup_timer = None
        self.cleanup_timer_exit = None
        
        self.position_home = [179.2, -42.1, 7.4, 186.7, 41.5, -1.6] #angle
        self.position_jig_A_grab = [-257.3, -138.3, 198, 68.3, 86.1, -47.0] #linear
        self.position_jig_B_grab = [-152.3, -129.0, 198, 4.8, 89.0, -90.7] #linear
        self.position_jig_C_grab = [-76.6, -144.6, 198, 5.7, 88.9, -50.1] #linear
        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59] #Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1] #Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.0, -25.6, -88.5, 95.8] #linear
        self.position_topping_A = [-200.3, 162.8, 359.9, -31.7, 87.8, 96.1] #Linear
        self.position_topping_B = [106.5, -39.7, 15.0, 158.7, 40.4, 16.9] #Angle
        self.position_topping_C = [43.6, 137.9, 350.1, -92.8, 87.5, 5.3] #Linear
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3] #Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2] #angle
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2] #Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6] #Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1] #Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7] #Linear
        
        self.this_action = '?' # 현재 동작
        self.actions = ['banana', 'choco', 'strawberry'] # 가능한 동작 리스트
        self.seq_length = 30 # 시퀀스 길이
        self.model = load_model('/home/lee/Desktop/Aris_Team5_build_test/src/robot_package/recog_model/model2_1.0.keras') # 동작 인식 모델
        self.seq = [] # 관절 데이터 시퀀스
        self.action_seq = [] # 동작 시퀀스

        self.mp_hands = mp.solutions.hands # 미디어파이프 손 모듈
        self.mp_drawing = mp.solutions.drawing_utils # 미디어파이프 그리기 도구
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.3,
            min_tracking_confidence=0.3
        )
        self.cap = cv2.VideoCapture(0) # 웹캠에서 비디오 캡처 시작
        pygame.mixer.init()
        # pygame.init()
        self.cleanup_timer_exit = None
        
        self.roi_defined = True
        self.hand_tracked = False
        self.roi_start = (160,180)
        self.roi_end = (480,400)
        
        points_dst = [(261, 285), (199, 291), (203, 357), (277, 355)]
        points_src = [(315, 191), (262, 193), (264, 239), (314, 235)]
        self.points_src = np.array(points_src, dtype=np.float32)
        self.points_dst = np.array(points_dst, dtype=np.float32)
        self.H, _ = cv2.findHomography(self.points_dst, self.points_src)
        
        self.solutions_trash = []
        self.img_width = 640
        self.img_height = 480
        self.exit_flag = False
        
    # =============================== init, error state ========================================= #
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        
        # self._arm.motion_enable(enable=True)
        # self._arm.set_gravity_direction([0, 0, 9.8])
        # self._arm.reset(wait=True)
        
        
        
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # ===============================  Register error/warn changed callback =============================== #
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # ===============================  Register state changed callback =============================== #
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # ===============================  Register count changed callback =============================== #
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    # ===============================  Register count changed callback =============================== #
    def _check_code(self, code, label):
        
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code,
                                                                                                    self._arm.connected,
                                                                                                    self._arm.state,
                                                                                                    self._arm.error_code,
                                                                                                    ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                        ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            # print("True --------")
            # print(self.alive)
            # print(self._arm.connected)
            # print(self._arm.error_code)
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4 # 만약 상태 값이 4보다 작다면 True, 그렇지 않으면 False를 반환. 즉 , self._arm.state이 0,1,2,3 일때만 true로 작동 가능한 상태
        else:
            # print("False --------")
            # print(self.alive)
            # print(self._arm.connected)
            # print(self._arm.error_code)
            return False

    # ===============================  fail sealing =============================== #
    def position_reverse_sealing_fail(self, linear_jig_position = [-257.3, -138.3, 192.1, 68.3, 86.1, -47.0]):
        reverse_position = linear_jig_position.copy()
        reverse_position[2] = reverse_position[2] - 10
        reverse_position[3] = -reverse_position[3]
        reverse_position[4] = -reverse_position[4]
        reverse_position[5] = reverse_position[5] - 180
        return reverse_position

    # ===============================  socket connecting =============================== #
    def socket_connect(self):

        self.HOST = '192.168.1.192'
        self.PORT = 20002
        self.BUFSIZE = 1024
        self.ADDR = (self.HOST, self.PORT)

        # self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.clientSocket.shutdown(1)
            self.clientSocket.close()
        except:
            pass

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # self
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.serverSocket.allow_reuse_address = True
        while True:
            try:
                self.serverSocket.bind(self.ADDR)
                print("bind")

                while True:
                    self.serverSocket.listen(1)
                    print(f'[LISTENING] Server is listening on robot_server')
                    time.sleep(1)
                    try:
                        while True:
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                print("socket accepted")
                                break
                            except:
                                time.sleep(1)
                                print('except')
                                # break

                        break

                    except socket.timeout:
                        print("socket timeout")

                    except:
                        pass
                break
            except:
                pass
        # self.clientSocket.settimeout(10.0)
        print("accept")
        print("--client info--")
        # print(self.clientSocket)

        self.connected = True
        self.state = 'ready'

        # ------------------- receive msg start -----------
        while self.connected:
            print('loop start')
            time.sleep(0.5)
            try:
                print('waiting')
                self.clientSocket.settimeout(10.0)
                self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                # try:
                #    self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                # except Exception as e:
                #    self.pprint('MainException: {}'.format(e))
                print('\n' + self.recv_msg)
                if self.recv_msg == '':
                    print('here')
                    # continue
                    # pass
                    # break
                    raise Exception('empty msg')
                self.recv_msg = self.recv_msg.split('/')

                if self.recv_msg[0] == 'app_ping':
                    # print('app_ping received')
                    send_msg = 'robot_ping'
                    now_temp = arm.temperatures
                    now_cur = arm.currents
                    send_msg = [
                        {
                            'type': 'A', 'joint_name': 'Base', 'temperature': now_temp[0],
                            'current': round(now_cur[0], 3) * 100
                        }, {
                            'type': 'B', 'joint_name': 'Shoulder', 'temperature': now_temp[1],
                            'current': round(now_cur[1], 3) * 100
                        }, {
                            'type': 'C', 'joint_name': 'Elbow', 'temperature': now_temp[2],
                            'current': round(now_cur[2], 3) * 100
                        }, {
                            'type': 'D', 'joint_name': 'Wrist1', 'temperature': now_temp[3],
                            'current': round(now_cur[3], 3) * 100
                        }, {
                            'type': 'E', 'joint_name': 'Wrist2', 'temperature': now_temp[4],
                            'current': round(now_cur[4], 3) * 100
                        }, {
                            'type': 'F', 'joint_name': 'Wrist3', 'temperature': now_temp[5],
                            'current': round(now_cur[5], 3) * 100
                        }
                    ]
                    try:
                        time.sleep(0.5)
                        self.clientSocket.send(f'{send_msg}'.encode('utf-8'))
                        print('robot_ping')

                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('ping send fail')
                    # send_msg = arm.temperatures
                    if self.state == 'ready':
                        print('STATE : ready for new msg')
                    else:
                        print('STATE : now moving')
                else:
                    self.recv_msg[0] = self.recv_msg[0].replace("app_ping", "")
                    if self.recv_msg[0] in ['breath', 'greet', 'farewell' 'dance_random', 'dance_a', 'dance_b',
                                            'dance_c',
                                            'sleep', 'comeon']:
                        print(f'got message : {self.recv_msg[0]}')
                        if self.state == 'ready':
                            self.state = self.recv_msg[0]
                    elif self.recv_msg[0] == 'robot_script_stop':
                        code = self._arm.set_state(4)
                        if not self._check_code(code, 'set_state'):
                            return
                        sys.exit()
                        self.is_alive = False
                        print('program exit')

                    # 픽업존 아이스크림 뺐는지 여부 확인
                    elif self.recv_msg[0].find('icecream_go') >= 0 or self.recv_msg[0].find(
                            'icecream_stop') >= 0 and self.state == 'icecreaming':
                        print(self.recv_msg[0])
                        if self.recv_msg[0].find('icecream_go') >= 0:
                            self.order_msg['makeReq']['latency'] = 'go'
                        else:
                            self.order_msg['makeReq']['latency'] = 'stop'
                            print('000000000000000000000000000000')

                    # 실링 존재 여부 확인

                    if self.recv_msg[0].find('sealing_pass') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'go'
                        print('socket_go')
                    elif self.recv_msg[0].find('sealing_reject') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'stop'
                        print('socket_stop')

                    else:
                        # print('else')
                        try:
                            self.order_msg = json.loads(self.recv_msg[0])
                            if self.order_msg['type'] == 'ICECREAM':
                                if self.state == 'ready':
                                    print('STATE : icecreaming')
                                    print(f'Order message : {self.order_msg}')
                                    self.state = 'icecreaming'
                            # else:
                            #    self.clientSocket.send('ERROR : already moving'.encode('utf-8'))
                            else:
                                self.clientSocket.send('ERROR : wrong msg received'.encode('utf-8'))
                        except:
                            pass
                self.recv_msg[0] = 'zzz'

            except Exception as e:
                self.pprint('MainException: {}'.format(e))
                # if e == 'empty msg' :
                #    pass
                # self.connected = False
                print('connection lost')
                while True:
                    time.sleep(2)
                    try:

                        try:
                            self.serverSocket.shutdown(socket.SHUT_RDWR)
                            self.serverSocket.close()
                        except:
                            pass

                        print('socket_making')
                        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                        self.serverSocket.bind(self.ADDR)
                        print("bind")

                        while True:
                            print('listening')
                            self.serverSocket.listen(1)
                            print(f'reconnecting')
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                break

                            except socket.timeout:
                                print('socket.timeout')
                                break

                            except:
                                pass
                        break
                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('except')
                        # pass

# ========================================================================   motion  part  ============================================================================== # 

    # =============== home =============== #
    def motion_home(self):
        code = self._arm.set_cgpio_analog(0, 0)
        print('home : _arm.set_cgpio_analog 0,0 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.set_cgpio_analog(1, 0)
        print('home : _arm.set_cgpio_analog 1,0 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # press_up
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        print('home => press up : _arm.set_cgpio_analog 3,0 ')
        if not self._check_code(code, 'set_cgpio_digital'):
            return

        # Joint Motion
        self._angle_speed = 80
        self._angle_acc = 200
        
        try:
            # self.clientSocket.send('motion_home_start'.encode('utf-8'))
            print('motion_home : try ')
        except:
            print('motion_home : except')
        print('motion_home start')
        
        # designed home
        # code = self._arm.set_servo_angle(angle=[179.0, -17.9, 17.7, 176.4, 61.3, 5.4], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=True, radius=10.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        
        code = self._arm.set_servo_angle(angle=self.position_home, speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        print("go to home.......")
        if not self._check_code(code, 'set_servo_angle'):
            return
        print('motion_home finish')
        # self.clientSocket.send('motion_home_finish'.encode('utf-8'))
        

    # =============== grab a capsule =============== # V
    def motion_grab_capsule(self):

        code = self._arm.set_cgpio_analog(0, 5)
        print('motion_grab_capsule : set_cgpio_analog : 0,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.set_cgpio_analog(1, 5)
        print('motion_grab_capsule : set_cgpio_analog : 1,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        '''
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        '''
        code = self._arm.stop_lite6_gripper()
        print('motion_grab_capsule : _arm.stop_lite6_gripper 1 => ok ')
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        try:
            # self.clientSocket.send('motion_grab_capsule_start'.encode('utf-8'))
            print('motion_grab_capsule : sending............ ')
        except:
            print('motion_grab_capsule : error')

        # code = self._arm.set_servo_angle(angle=[175.4, 28.7, 23.8, 84.5, 94.7, -5.6], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return
        
        if self.order_msg['makeReq']['jigNum'] in ['A']:
            # code = self._arm.set_servo_angle(angle=[166.1, 30.2, 25.3, 75.3, 93.9, -5.4], speed=self._angle_speed,
            #                                  mvacc=self._angle_acc, wait=True, radius=0.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #     return
            pass
            print('A WHAT???')
        else:

            code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            print('Hmm.............. A?..')
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_servo_angle(angle=[166.1, 30.2, 25.3, 75.3, 93.9, -5.4], speed=self._angle_speed,
            #                                  mvacc=self._angle_acc, wait=False, radius=20.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #     return


        code = self._arm.open_lite6_gripper()
        print('_arm.stop_lite6_gripper 2 : ok ')
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        if self.order_msg['makeReq']['jigNum'] == 'A':
            code = self._arm.set_servo_angle(angle=[179.5, 33.5, 32.7, 113.0, 93.1, -2.3], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            print("move to A!!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_position(*[-255.4, -139.3, 193.5, -12.7, 87.2, -126.1], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to A JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'B':

            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to B JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'C':
            code = self._arm.set_servo_angle(angle=[182.6, 27.8, 27.7, 55.7, 90.4, -6.4], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            print("move to C !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_position(*[-76.6, -144.6, 194.3, 5.7, 88.9, -50.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to C JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.close_lite6_gripper()
        print('_arm.stop_lite6_gripper 3 : ok ')
        if not self._check_code(code, 'close_lite6_gripper'):
            return

        time.sleep(1)
        
        if self.order_msg['makeReq']['jigNum'] == 'C':
            code = self._arm.set_position(z=150, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            print("move to C 2 !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return
            self._tcp_speed = 200
            self._tcp_acc = 1000
            code = self._arm.set_tool_position(*[0.0, 0.0, -90.0, 0.0, 0.0, 0.0], speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, wait=False)
            print("TOOL !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return
        else:
            code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            print("TOOL2 !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        self._angle_speed = 180
        self._angle_acc = 500

        if self.order_msg['makeReq']['sealing'] in ['yes']:
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            print("yes")
            if not self._check_code(code, 'set_servo_angle'):
                return
        else :
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            print("no")
            if not self._check_code(code, 'set_servo_angle'):
                return
        try:
            self.clientSocket.send('motion_grab_capsule_finish'.encode('utf-8'))
        except:
            print('socket error')
    
    # =============== check a sealing =============== #
    def motion_check_sealing(self):
        print('sealing check')
        self._angle_speed = 200
        self._angle_acc = 200
        # self.clientSocket.send('motion_sheck_sealing'.encode('utf-8'))
        code = self._arm.set_position(*self.position_sealing_check, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
    
    # =============== grab a capsule => fail  =============== # V
    def motion_place_fail_capsule(self):

        # code = self._arm.set_servo_angle(angle=[154.2, -3.3, 13.7, 101.2, 83.4, 130.4], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #    return


        if self.order_msg['makeReq']['jigNum'] == 'A':
            code = self._arm.set_servo_angle(angle=[177.3, 5.5, 12.9, 133.6, 81.3, 183.5], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_A_grab), speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'B':
            code = self._arm.set_servo_angle(angle=[159.5, 11.8, 22.2, 75.6, 92.8, 186.6], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_B_grab) , speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'C':
            code = self._arm.set_servo_angle(angle=[176.9, -2.2, 15.3, 69.3, 87.5, 195.5], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_C_grab) , speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return

    # =============== place a capsule =============== #
    def motion_place_capsule(self):
        code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[8.4, -42.7, 23.7, 177.4, 31.6, 3.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_servo_angle(angle=[8.4, -32.1, 55.1, 96.6, 29.5, 81.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_position(*self.position_before_capsule_place, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*self.position_capsule_place, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(2)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

    # =============== grab a cup =============== # V
    def motion_grab_cup(self):
        try:
            self.clientSocket.send('motion_grab_cup_start'.encode('utf-8'))
        except:
            print('socket error')

        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        if self.order_msg['makeReq']['cupNum'] in ['A', 'B']:
            code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            # code = self._arm.set_position(*[193.8, -100.2, 146.6, 135.9, -86.0, -55.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=10.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_position(*[195.0, -96.5, 145.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            # code = self._arm.set_position(*[195.5, -96.6, 145.6, 179.0, -87.0, -97.1], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            # code = self._arm.set_position(*[214.0, -100.2, 145.0, -25.6, -88.5, 95.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        try:
            self.clientSocket.send('motion_grab_cup_finish'.encode('utf-8'))
        except:
            print('socket error')

        time.sleep(0.5)

    # =============== topping =============== # V
    def motion_topping(self):
        try:
            self.clientSocket.send('motion_topping_start'.encode('utf-8'))
        except:
            print('socket error')

        print('send')

        if self.order_msg['makeReq']['topping'] == '1':
            code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            if self.order_msg['makeReq']['jigNum'] == 'C':
                code = self._arm.set_position(*self.position_topping_C, speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                                wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 3)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(3)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return

                code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                                relative=True, wait=False)
                if not self._check_code(code, 'set_position'):
                    return

            elif self.order_msg['makeReq']['jigNum'] in ['B']:
                code = self._arm.set_servo_angle(angle=[55.8, -48.2, 14.8, 86.1, 60.2, 58.7], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=False, radius=20.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                # code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
                #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                # if not self._check_code(code, 'set_servo_angle'):
                #    return
                code = self._arm.set_servo_angle(angle=self.position_topping_B, speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                                wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 4)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(4)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                                relative=True, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=False, radius=10.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return

            elif self.order_msg['makeReq']['jigNum'] == 'A':
                code = self._arm.set_position(*self.position_topping_A, speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_servo_angle(angle=[130.0, -33.1, 12.5, 194.3, 51.0, 0.0], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_position(*[-38.2, 132.2, 333.9, -112.9, 86.3, -6.6], speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
            # code = self._arm.set_position(*[165.1, 162.9, 362.5, -31.7, 86.6, 9.5], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        else:
            # code = self._arm.set_servo_angle(angle=[45.8, -17.9, 33.5, 186.9, 41.8, -7.2], speed=self._angle_speed,
            #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #    return
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        try:
            self.clientSocket.send('motion_topping_finish'.encode('utf-8'))
        except:
            print('socket error')

        time.sleep(0.5)

    # =============== make a icecream =============== #step a,b,c
    def motion_make_icecream(self):
        try:
            self.clientSocket.send('motion_make_icecream_start'.encode('utf-8'))
        except:
            print('socket error')
        if self.order_msg['makeReq']['topping'] == '1':
            time.sleep(5)
        else:
            time.sleep(8)
        try:
            self.clientSocket.send('motion_icecreaming_1'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(4)
        code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        try:
            self.clientSocket.send('motion_icecreaming_2'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(4)
        code = self._arm.set_position(z=-10, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        if not self._check_code(code, 'set_pause_time'):
            return
        try:
            self.clientSocket.send('motion_icecreaming_3'.encode('utf-8'))
        except:
            print('socket error')
        code = self._arm.set_position(z=-50, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        try:
            self.clientSocket.send('motion_make_icecream_finish'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(0.5)

    # =============== serving =============== # v
    def motion_serve(self):
        try:
            self.clientSocket.send('motion_serve_start'.encode('utf-8'))
        except:
            print('socket error')
            
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

        if self.order_msg['makeReq']['jigNum'] == 'A':
            # code = self._arm.set_position(*[-251.2, -142.1, 213.7, -28.1, 88.8, -146.0], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            # code = self._arm.set_position(*[-250.3, -138.3, 213.7, 68.3, 86.1, -47.0], speed=self._tcp_speed,
            #                              mvacc=self._tcp_acc, radius=0.0, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*self.position_jig_A_serve, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(z=-18, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-256.2, -126.6, 210.1, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-242.8, -96.3, 210.5, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_tool_position(*[0.0, 0.0, -30, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[-189.7, -26.0, 193.3, -28.1, 88.8, -146.0], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif self.order_msg['makeReq']['jigNum'] == 'B':

            code = self._arm.set_position(*self.position_jig_B_serve, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(z=-13, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-165.0, -122.7, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-165.9, -81.9, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_tool_position(*[0.0, 0.0, -30, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[-168.5, -33.2, 192.8, -92.9, 86.8, -179.3], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        elif self.order_msg['makeReq']['jigNum'] == 'C':
            # code = self._arm.set_servo_angle(angle=[171.0, 13.7, 13.5, 73.9, 92.3, -2.9], speed=self._angle_speed,
            #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
            # if not self._check_code(code, 'set_servo_angle'):
            #    return
            code = self._arm.set_servo_angle(angle=[177.6, 0.2, 13.5, 70.0, 94.9, 13.8], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_jig_C_serve, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(z=-12, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-75, -132.8, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-92.0, -107.5, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            # code = self._arm.set_tool_position(*[0.0, 0.0, -30, 0.0, 0.0, 0.0], speed=self._tcp_speed, mvacc=self._tcp_acc, wait=True)
            # if not self._check_code(code, 'set_position'):
            #    return
            code = self._arm.set_position(*[-98.1, -52.1, 191.4, -68.4, 86.4, -135.0], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        try:
            self.clientSocket.send('motion_serve_finish'.encode('utf-8'))
        except:
            print('socket error')
        time.sleep(0.5)
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

    # =============== trash a capsule =============== # hmm....... ?
    def motion_trash_capsule(self):
        self._angle_speed = 150
        self._angle_acc = 300
        
        code = self._arm.set_servo_angle(angle=[51.2, -8.7, 13.8, 95.0, 86.0, 17.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[-16.2, -19.3, 42.7, 82.0, 89.1, 55.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        
        code = self._arm.set_servo_angle(angle=[-19.9, -19.1, 48.7, 87.2, 98.7, 60.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_position(*[222.8, 0.9, 470.0, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(*self.position_capsule_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        
        time.sleep(1)
        
        code = self._arm.set_position(z=30, radius=-1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        self._tcp_speed = 100
        self._tcp_acc = 1000
        
        code = self._arm.set_position(*[221.9, -5.5, 500.4, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        self._angle_speed = 60
        self._angle_acc = 100
        
        code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 50.4, 78.1, 63.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        self._angle_speed = 160
        self._angle_acc = 1000
        
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        
        self._angle_speed = 120
        self._angle_acc = 1000
        
        code = self._arm.set_servo_angle(angle=[28.3, -9.0, 12.6, 85.9, 78.5, 20.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
    
        code = self._arm.set_servo_angle(angle=[149.3, -9.4, 10.9, 114.7, 69.1, 26.1], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        time.sleep(0.5)

    # =============== dance_a =============== #
    # designed 'poke'
    def motion_dance_a(self):
        try:
            self.clientSocket.send('dance_a_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 60
        self._angle_acc = 300
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[212.0, -21.0, 112.0, 207.0, -0.8, 7.3], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[212.0, -38.0, 100.3, 180.4, -6.4, 6.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        '''
        code = self._arm.set_servo_angle(angle=[329.0, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[329.0, -21.0, 112.0, 207.0, -0.8, 7.3], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[329.0, -38.0, 100.3, 180.4, -6.4, 6.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        '''
        self._angle_speed = 60
        self._angle_acc = 200
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    # =============== dance_b =============== #
    # designed 'shake'
    def motion_dance_b(self):
        try:
            self.clientSocket.send('dance_b_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 70
        self._angle_acc = 200
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(4)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[220.7, -39.1, 67.0, 268.3, -40.0, -91.8], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[183.0, -39.1, 102.7, 220.0, -11.6, -140.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    # =============== dance_c =============== #
    # designed '빙글빙글'
    def motion_dance_c(self):  
        try:
            self.clientSocket.send('dance_c_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 150
        self._angle_acc = 700
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            t1 = time.monotonic()
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 250.0, 173.1, 0.0, -135.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, -70.0, 110.0, 180.0, 0.0, 135.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            interval = time.monotonic() - t1
            if interval < 0.01:
                time.sleep(0.01 - interval)
        code = self._arm.set_servo_angle(angle=[180.0, 70.0, 250.0, 173.1, 0.0, -135.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('dance_c_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    # =============== dance_come_on =============== #
    # designed '컴온컴온
    def motion_come_on(self):
        try:
            self.clientSocket.send('comeon_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 80
        self._angle_acc = 400
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[180.0, 70.0, 220.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(2)):
            if not self.is_alive:
                break
            t1 = time.monotonic()
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 220.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 62.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 55.0, 222.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 45.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 35.0, 224.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 25.0, 224.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 15.0, 226.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 5.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 0.0, 228.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 5.0, 230.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 20.0, 226.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 35.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 45.0, 228.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 55.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 65.0, 224.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            interval = time.monotonic() - t1
            if interval < 0.01:
                time.sleep(0.01 - interval)
        code = self._arm.set_servo_angle(angle=[180.0, 65.0, 222.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('comeon_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    # =============== motion_greeting => HI! =============== #
    def motion_greet(self):
        # try:
        #     self.clientSocket.send('greet_start'.encode('utf-8'))
        # except:
        #     print('socket error')

        self._angle_speed = 100
        self._angle_acc = 350

        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 181.5, -1.9, -92.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        # try:
        #     self.clientSocket.send('motion_greet finish'.encode('utf-8'))
        # except:
        #     print('socket error')
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 181.5, -1.9, -92.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        # while True:
        #     try:
        #         self.clientSocket.send('motion_greet_finish'.encode('utf-8'))
        #         break
        #     except:
        #         print('socket error')

    # =============== motion_breath  =============== #
    def motion_breath(self):
        pass

    # =============== motion_sleep =============== #
    # designed 'sleep'
    def motion_sleep(self): 
        try:
            self.clientSocket.send('sleep_start'.encode('utf-8'))
        except:
            print('socket error')

        for i in range(int(1)):
            if not self.is_alive:
                break
            for i in range(int(2)):
                if not self.is_alive:
                    break
                self._angle_speed = 20
                self._angle_acc = 200
                code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                self._angle_speed = 5
                self._angle_acc = 5
                code = self._arm.set_servo_angle(angle=[179.0, -10.2, 24.0, 178.2, 39.2, -2.0], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
            self._angle_speed = 30
            self._angle_acc = 300
            code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            for i in range(int(3)):
                if not self.is_alive:
                    break
                self._angle_speed = 180
                self._angle_acc = 1000
                code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 199.8, 43.4, -11.0],
                                                    speed=self._angle_speed, mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 157.3, 43.2, 12.7], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
            self._angle_speed = 20
            self._angle_acc = 200
            code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_pause_time(2)
            if not self._check_code(code, 'set_pause_time'):
                return
        while True:
            try:
                self.clientSocket.send('sleep_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    # =============== motion_cleaning =============== #
    def motion_clean_mode(self):
        pass

    # =============== pin_off =============== #
    def pin_off(self):
        self.clientSocket.send('pin_off_start'.encode('utf-8'))
        # cup_dispenser_up
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        # press_up
        code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        self.clientSocket.send('pin_off_finish'.encode('utf-8'))

    # =============== pin_test =============== #
    def pin_test(self):
        time.sleep(3)
        code = self._arm.set_servo_angle(angle=[179.0, -17.7, 29.0, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        time.sleep(2)
        code = self._arm.set_servo_angle(angle=[179.0, -17.7, 83.3, 177.8, 43.8, -1.4], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(3)
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(3)
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

    # =============== joint_infomation =============== #
    def joint_state(self):
        while self.is_alive:
            print(f'joint temperature : {arm.temperatures}')
            time.sleep(0.5)
            print(f'joint current : {arm.currents}')
            time.sleep(10)

    # =============== joint_infomation_ros =============== #
    def joint_state_ros(self):
        joint_temperatures = self.arm.temperatures  # arm 객체의 temperatures 속성 사용
        print(f'joint temperature : {joint_temperatures}')
        time.sleep(0.5)
        joint_currents = self.arm.currents  # arm 객체의 currents 속성 사용
        print(f'joint current : {joint_currents}')
        return joint_currents, joint_temperatures


    # ============================== tunning code ============================== #
    def motion_grab_capsule_a(self):
            
        order_msg_A = 'A'
        # order_msg_A_1 = 'aa'
        msg_A = 'a'
        
        code = self._arm.set_cgpio_analog(0, 5)
        print('motion_grab_capsule : set_cgpio_analog : 0,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.set_cgpio_analog(1, 5)
        print('motion_grab_capsule : set_cgpio_analog : 1,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.stop_lite6_gripper()
        print('motion_grab_capsule : _arm.stop_lite6_gripper 1 => ok ')
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        
        # if order_msg_A_1 == 'aa':
        #     pass
        #     print('A WHAT???')
        # else:
        #     code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
        #                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        #     print('Hmm.............. A?..')
        #     if not self._check_code(code, 'set_servo_angle'):
        #         return


        # code = self._arm.open_lite6_gripper()
        # print('_arm.stop_lite6_gripper 2 : ok ')
        # if not self._check_code(code, 'open_lite6_gripper'):
        #     return
        # time.sleep(1)


        if order_msg_A == 'A':
            code = self._arm.set_servo_angle(angle=[179.5, 33.5, 32.7, 113.0, 93.1, -2.3], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            print("move to A!!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to A JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return


        # 공통 그리퍼 동작
        code = self._arm.close_lite6_gripper()
        print('_arm.stop_lite6_gripper 3 : ok ')
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)


        self._angle_speed = 180
        self._angle_acc = 500

        if msg_A == 'a':
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            print("yes")
            if not self._check_code(code, 'set_servo_angle'):
                return
        else :
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            print("no")
            if not self._check_code(code, 'set_servo_angle'):
                return

    def motion_grab_capsule_b(self):
        order_msg_B = 'B'
        msg_B = 'b'
    
        code = self._arm.set_cgpio_analog(0, 5)
        print('motion_grab_capsule : set_cgpio_analog : 0,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.set_cgpio_analog(1, 5)
        print('motion_grab_capsule : set_cgpio_analog : 1,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.stop_lite6_gripper()
        print('motion_grab_capsule : _arm.stop_lite6_gripper 1 => ok ')
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)


        if order_msg_B == 'B':

            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to B JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.close_lite6_gripper()
        print('_arm.stop_lite6_gripper 3 : ok ')
        if not self._check_code(code, 'close_lite6_gripper'):
            return

        time.sleep(1)

        self._angle_speed = 180
        self._angle_acc = 500

        if msg_B == 'b':
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            print("yes")
            if not self._check_code(code, 'set_servo_angle'):
                return
        else :
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            print("no")
            if not self._check_code(code, 'set_servo_angle'):
                return

    def motion_grab_capsule_c(self):
        order_msg_C = 'C'
        msg_C = 'c'
        msg_C_1 = 'cc'
    
        code = self._arm.set_cgpio_analog(0, 5)
        print('motion_grab_capsule : set_cgpio_analog : 0,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.set_cgpio_analog(1, 5)
        print('motion_grab_capsule : set_cgpio_analog : 1,5 ')
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)

        code = self._arm.stop_lite6_gripper()
        print('motion_grab_capsule : _arm.stop_lite6_gripper 1 => ok ')
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

    
        if order_msg_C == 'C':
            code = self._arm.set_servo_angle(angle=[182.6, 27.8, 27.7, 55.7, 90.4, -6.4], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            print("move to C !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_servo_angle'):
                return

            code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to C JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.close_lite6_gripper()
        print('_arm.stop_lite6_gripper 3 : ok ')
        if not self._check_code(code, 'close_lite6_gripper'):
            return

        time.sleep(1)
        
        if msg_C == 'C':
            code = self._arm.set_position(z=150, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            print("move to C 2 !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return
            self._tcp_speed = 200
            self._tcp_acc = 1000
            code = self._arm.set_tool_position(*[0.0, 0.0, -90.0, 0.0, 0.0, 0.0], speed=self._tcp_speed,
                                                mvacc=self._tcp_acc, wait=False)
            print("TOOL !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return
        else:
            code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            print("TOOL2 !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return


        self._angle_speed = 180
        self._angle_acc = 500

        if msg_C_1 == 'cc':
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            print("yes")
            if not self._check_code(code, 'set_servo_angle'):
                return
        else:
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            print("no")
            if not self._check_code(code, 'set_servo_angle'):
                    return

    def motion_grab_capsule_x(self):
                
            order_msg_A = 'A'
            # order_msg_A_1 = 'aa'
            msg_A = 'a'
            
            code = self._arm.set_cgpio_analog(0, 5)
            print('motion_grab_capsule : set_cgpio_analog : 0,5 ')
            if not self._check_code(code, 'set_cgpio_analog'):
                return
            
            code = self._arm.set_cgpio_analog(1, 5)
            print('motion_grab_capsule : set_cgpio_analog : 1,5 ')
            if not self._check_code(code, 'set_cgpio_analog'):
                return

            # Joint Motion
            self._angle_speed = 100
            self._angle_acc = 100

            self._tcp_speed = 100
            self._tcp_acc = 1000

            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(1)
            
            code = self._arm.stop_lite6_gripper()
            print('motion_grab_capsule : _arm.stop_lite6_gripper 1 => ok ')
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)

            
            # if order_msg_A_1 == 'aa':
            #     pass
            #     print('A WHAT???')
            # else:
            #     code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
            #                                         mvacc=self._angle_acc, wait=True, radius=0.0)
            #     print('Hmm.............. A?..')
            #     if not self._check_code(code, 'set_servo_angle'):
            #         return


            # code = self._arm.open_lite6_gripper()
            # print('_arm.stop_lite6_gripper 2 : ok ')
            # if not self._check_code(code, 'open_lite6_gripper'):
            #     return
            # time.sleep(1)


            # if order_msg_A == 'A':
            #     code = self._arm.set_servo_angle(angle=[179.5, 33.5, 32.7, 113.0, 93.1, -2.3], speed=self._angle_speed,
            #                                         mvacc=self._angle_acc, wait=False, radius=20.0)
            #     print("move to A!!!!!!!!!!!!!!!!")
            #     if not self._check_code(code, 'set_servo_angle'):
            #         return

            #     code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
            #                                     mvacc=self._tcp_acc, radius=0.0, wait=True)
            #     print("move to A JIG !!!!!!!!!!!!!!!!")
            #     if not self._check_code(code, 'set_position'):
            #         return


            self._angle_speed = 180
            self._angle_acc = 500

            if msg_A == 'a':
                code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=False, radius=30.0)
                print("yes")
                if not self._check_code(code, 'set_servo_angle'):
                    return
            else :
                code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
                print("no")
                if not self._check_code(code, 'set_servo_angle'):
                    return

    def motion_grab_capsule_b_down(self):
        
        order_msg_B = 'B'
        msg_B = 'b'
    
        # code = self._arm.set_cgpio_analog(0, 5)
        # print('motion_grab_capsule : set_cgpio_analog : 0,5 ')
        # if not self._check_code(code, 'set_cgpio_analog'):
        #     return
        
        # code = self._arm.set_cgpio_analog(1, 5)
        # print('motion_grab_capsule : set_cgpio_analog : 1,5 ')
        # if not self._check_code(code, 'set_cgpio_analog'):
        #     return

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.stop_lite6_gripper()
        print('motion_grab_capsule : _arm.stop_lite6_gripper 1 => ok ')
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
                                                    mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return


        if order_msg_B == 'B':
            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            print("move to B JIG !!!!!!!!!!!!!!!!")
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.close_lite6_gripper()
        print('_arm.stop_lite6_gripper 3 : ok ')
        if not self._check_code(code, 'close_lite6_gripper'):
            return

        time.sleep(1)

        self._angle_speed = 180
        self._angle_acc = 500

        if msg_B == 'b':
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=30.0)
            print("yes")
            if not self._check_code(code, 'set_servo_angle'):
                return
        else :
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            print("no")
            if not self._check_code(code, 'set_servo_angle'):
                return

    def motion_serve_a(self):
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

        
        code = self._arm.set_position(*self.position_jig_A_serve, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(z=-18, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.set_position(*[-256.2, -126.6, 210.1, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        
        time.sleep(0.5)
        code = self._arm.set_position(*[-242.8, -96.3, 210.5, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(*[-189.7, -26.0, 193.3, -28.1, 88.8, -146.0], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        time.sleep(0.5)
        
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_serve_b(self):
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

        code = self._arm.set_position(*self.position_jig_B_serve, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(z=-13, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-165.0, -122.7, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        code = self._arm.set_position(*[-165.9, -81.9, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(*[-168.5, -33.2, 192.8, -92.9, 86.8, -179.3], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        time.sleep(0.5)
        
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_serve_c(self):
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 25 #100
        self._tcp_acc = 250 #1000

        code = self._arm.set_servo_angle(angle=[177.6, 0.2, 13.5, 70.0, 94.9, 13.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*self.position_jig_C_serve, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(z=-12, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-75, -132.8, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        code = self._arm.set_position(*[-92.0, -107.5, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.set_position(*[-98.1, -52.1, 191.4, -68.4, 86.4, -135.0], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        time.sleep(0.5)
        
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_serve_c_x(self):
        # code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=False, radius=20.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        
        # code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return

        self._tcp_speed = 100 #100
        self._tcp_acc = 1000 #1000

        # code = self._arm.set_servo_angle(angle=[177.6, 0.2, 13.5, 70.0, 94.9, 13.8], speed=self._angle_speed,
        #                                     mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_position(*self.position_jig_C_serve, speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        # code = self._arm.set_position(z=-12, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
        #                                 wait=True)
        # if not self._check_code(code, 'set_position'):
            # return
            
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        
        time.sleep(1)
        # code = self._arm.set_position(*[-75, -132.8, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return
        
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
        # code = self._arm.set_position(*[-92.0, -107.5, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return

        # code = self._arm.set_position(*[-98.1, -52.1, 191.4, -68.4, 86.4, -135.0], speed=self._tcp_speed,
        #                                 mvacc=self._tcp_acc, radius=0.0, wait=True)
        # if not self._check_code(code, 'set_position'):
        #     return

        time.sleep(0.5)
        
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

    def motion_topping_a(self):
        a = 1
        if a == 1:
            code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            code = self._arm.set_position(*self.position_topping_A, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            
            code = self._arm.set_pause_time(1)
            if not self._check_code(code, 'set_pause_time'):
                return
            
            code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            
            code = self._arm.set_servo_angle(angle=[130.0, -33.1, 12.5, 194.3, 51.0, 0.0], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            code = self._arm.set_position(*[-38.2, 132.2, 333.9, -112.9, 86.3, -6.6], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=10.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=10.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            time.sleep(8)
            code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
        
        else:

            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            
            code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
        time.sleep(0.5)

    def motion_topping_b(self):
        b = 1
        if b == 1:
            code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            code = self._arm.set_servo_angle(angle=[55.8, -48.2, 14.8, 86.1, 60.2, 58.7], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
            code = self._arm.set_servo_angle(angle=self.position_topping_B, speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            # code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(4)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                            relative=True, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=False, radius=10.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=10.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return        
            code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
    
            time.sleep(8)
            code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
        else:

            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
        time.sleep(0.5)

    def motion_topping_c(self):
        c = 1
        if c == 1:
            code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_topping_C, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                            wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_pause_time(1)
            # code = self._arm.set_pause_time(int(self.order_msg['makeReq']['toppingAmount']) - 3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_pause_time(3)
            if not self._check_code(code, 'set_pause_time'):
                return
            code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return

            code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                            relative=True, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
                                            mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            
            time.sleep(8)
            code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
        
        else:
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
                                                mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            
        time.sleep(0.5)

    def motion_grab_cup_a(self):
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        
        time.sleep(1)

        # a, b
        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=10.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
            
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        time.sleep(0.5)

    def motion_grab_cup_b(self):
        
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        
        time.sleep(1)

        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=10.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        
        time.sleep(2)

        code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return


        time.sleep(0.5)

    def motion_grab_cup_c(self):
        
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        
        time.sleep(1)

        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=10.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        
        code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
                return
            
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        
        time.sleep(2)

        code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return


        time.sleep(0.5)

    def motion_place_fail_capsule_a(self):

        code = self._arm.set_servo_angle(angle=[177.3, 5.5, 12.9, 133.6, 81.3, 183.5], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_A_grab), speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
            
            
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return

    def motion_place_fail_capsule_b(self):

        code = self._arm.set_servo_angle(angle=[159.5, 11.8, 22.2, 75.6, 92.8, 186.6], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_B_grab) , speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return


        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return

    def motion_place_fail_capsule_c(self):

        code = self._arm.set_servo_angle(angle=[176.9, -2.2, 15.3, 69.3, 87.5, 195.5], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_C_grab) , speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=False)
        if not self._check_code(code, 'set_position'):
            return

    def motion_make_icecream_a(self):
        time.sleep(5)

        code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return

        time.sleep(2)

        
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        time.sleep(0.5)

    def motion_make_icecream_b(self):
        time.sleep(5)
        
        code = self._arm.set_position(z=-10, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
    
        if not self._check_code(code, 'set_position'):
            return
        
        
        time.sleep(2)
        
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        time.sleep(0.5)

    def motion_make_icecream_c(self):
        time.sleep(5)
                
        code = self._arm.set_position(z=-50, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                        wait=True)
        if not self._check_code(code, 'set_position'):
            return
        
        time.sleep(1)
        
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        time.sleep(0.5)

    def make_icecream_half(self):
        time.sleep(5)
    
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        
        time.sleep(0.5)

    def apocato_make(self):
        try:
            code = self._arm.set_position(*[-164.1, 1.9, 165.7, 145.4, 82.9, -35.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-234.0, 9.9, 168.6, 143.3, 87.1, -37.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(3)
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(3)
            code = self._arm.set_position(*[-221.6, 6.8, 345.8, 3.0, 83.7, -177.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-256.9, -93.0, 269.6, -123.0, 83.5, 89.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-254.9, -111.8, 287.0, -80.4, 81.2, 137.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-255.2, -111.4, 287.0, -88.0, 39.5, 128.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-253.2, -113.9, 286.3, -89.1, 7.5, 130.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-247.2, -120.0, 284.3, -91.2, -25.2, 139.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-253.0, -113.9, 288.8, -32.6, 86.2, -171.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-158.0, -1.2, 239.7, -13.7, 87.3, 165.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-240.1, 11.4, 172.0, 139.0, 88.2, -42.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            code = self._arm.set_position(*[-153.5, 5.9, 156.9, 164.0, 81.8, -16.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-157.1, 6.8, 302.0, 8.5, 82.2, -173.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
        except:
            pass

    def speak(self,text):
        tts = gTTS(text=text, lang='ko')
        self.filename='voice.mp3'
        tts.save(self.filename)
        playsound(self.filename)

    def play_audio(self, audio_file):
        playsound(audio_file)
        
        
    def play_audio_game(self, game_file):
        pygame.mixer.music.load(game_file)
        pygame.mixer.music.play(-1)  # 반복
        # while not self.stop_event.is_set():
        #     time.sleep(1)  # 짧은 대기 시간으로 반복 체크
            
        pygame.mixer.music.stop()  # 음악 정지
        pygame.quit()  # pygame 종료

        pygame.mixer.music.stop()

    def recognize_speech(self):
        r = sr.Recognizer()
        
        with sr.Microphone() as source:
            print("Say Something")
            speech = r.listen(source)
        try:
            audio = r.recognize_google(speech, language="ko-KR")
            print("Your speech thinks like\n " + audio)
            return audio
        
        except sr.UnknownValueError:
            print("Your speech can not understand")
            return None
        
        except sr.RequestError as e:
            print("Request Error!; {0}".format(e))
            return None

    def recognize_speech_with_apagato(self):
        audio = self.recognize_speech()
        if audio and "아포가토" in audio:
            print("아포가토가 포함된 음성이 인식되었습니다.")
            self.apocato()
        else:
            print("아포가토가 포함되지 않은 음성이 인식되었습니다.")

    def motion_half_grap_cup_a(self):
        try:
            code = self._arm.set_position(*[-170.6, -60.4, 199.2, 167.6, 82.5, 39.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-218.8, -97.0, 185.0, -178.3, 87.5, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-205.6, -112.8, 181.9, -131.6, 88.5, 99.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-244.6, -136.0, 196.1, -174.7, 80.7, 64.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-244.7, -136.8, 203.5, 178.1, 87.7, 57.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(3)
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(3)
            code = self._arm.set_position(*[-187.5, -104.8, 367.8, -18.1, 84.2, -148.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-150.2, 52.7, 307.7, 47.8, 78.5, -145.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-61.1, 134.9, 320.5, 12.5, 84.7, 119.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[12.4, 139.0, 329.6, -48.3, 83.2, 31.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[95.0, 102.2, 328.2, -42.2, 85.7, -1.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[127.6, 135.0, 331.4, -107.0, 86.8, -69.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[159.1, 168.6, 352.2, -164.2, 84.6, -125.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[159.8, 168.1, 359.8, -26.9, 87.9, 11.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            time.sleep(8)
            code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            time.sleep(1)
            
            code = self._arm.set_position(*[140.5, 152.4, 302.8, -125.6, 87.2, -83.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[68.9, 138.7, 300.3, -13.7, 85.8, 46.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[20.1, 141.6, 318.8, -40.6, 82.9, 41.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-107.4, 94.0, 317.4, -65.5, 80.0, 78.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-144.9, -33.6, 307.3, -62.5, 86.9, 134.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-214.2, -110.3, 253.3, 149.7, 84.0, 20.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-248.1, -135.3, 198.0, 92.7, 85.6, -28.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-246.8, -135.9, 195.5, -148.0, 87.0, 92.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(3)
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(3)
            code = self._arm.set_position(*[-194.4, -105.3, 203.1, 165.8, 83.4, 35.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-171.4, -5.1, 310.6, 35.7, 80.3, -143.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def motion_half_grap_cup_c(self):
        try:
            code = self._arm.set_position(*[-171.4, -5.1, 310.6, 35.7, 80.3, -143.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-142.4, -53.6, 225.6, -79.5, 88.8, 164.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-126.2, -83.4, 225.0, 132.9, 81.9, 70.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-108.8, -101.3, 203.5, 122.3, 86.7, 67.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-90.7, -132.5, 198.4, -174.0, 83.0, 133.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-87.7, -137.4, 198.0, -173.0, 82.7, 134.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-82.7, -143.0, 195.0, -171.0, 79.6, 137.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-83.7, -144.9, 204.7, 128.8, 88.1, 75.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(3)
            code = self._arm.close_lite6_gripper()
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(3)
            code = self._arm.set_position(*[-85.8, -146.5, 242.2, -177.4, 86.7, 130.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-165.2, -87.4, 217.6, 14.2, 84.0, -77.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-189.1, -27.8, 330.4, 27.7, 74.1, -144.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-88.8, 136.5, 311.8, -77.2, 75.5, 43.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-23.6, 117.9, 346.7, -27.8, 82.0, 73.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[63.8, 102.4, 340.2, -73.4, 86.8, -16.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[156.1, 145.1, 322.0, -135.2, 83.9, -94.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[172.2, 164.5, 349.1, -44.8, 85.3, -1.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[165.2, 163.2, 362.5, -101.5, 87.2, -54.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            time.sleep(12)
            code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            time.sleep(1)
            
            code = self._arm.set_position(*[152.8, 151.9, 317.7, -164.3, 86.2, -117.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[74.7, 135.6, 330.3, -174.2, 82.9, -111.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-11.2, 141.5, 349.6, -86.4, 85.5, 10.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-123.5, 70.1, 349.1, -91.4, 85.0, 60.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-165.0, -45.3, 295.5, 28.0, 88.0, -135.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-135.9, -97.3, 223.6, 109.7, 83.1, 19.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-145.6, -124.7, 204.0, 52.4, 86.6, -43.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-143.3, -123.2, 193.5, -21.9, 88.8, -118.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(3)
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(3)
            code = self._arm.set_position(*[-142.4, -123.0, 187.6, -146.4, 84.7, 118.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-147.5, -47.6, 189.0, -126.9, 87.9, 138.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            
            # code = self._arm.set_position(*[-183.3, -3.2, 299.6, 38.7, 82.0, -141.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            # if not self._check_code(code, 'set_position'):
            #     return
            # time.sleep(2)
            
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def sub_motion_grap_cup_a(self):
        try:
            code = self._arm.set_position(*[-140.4, -36.5, 266.0, -62.2, 86.1, -157.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-184.5, 6.2, 295.6, 55.9, 75.0, -120.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-36.1, 148.1, 317.9, -65.0, 84.8, 34.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[178.9, 158.0, 339.5, -79.6, 82.2, -40.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[236.4, 52.6, 351.3, -18.5, 83.1, -11.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[267.3, 1.2, 520.9, 18.7, 85.3, 18.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[262.0, 26.6, 513.7, -97.2, 74.7, -72.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[214.5, 19.1, 484.7, -121.7, 85.0, -60.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def start_server(self):
        host = '192.168.1.21'  # 모든 IP 주소에서 연결을 허용
        port = 12345
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server_socket.settimeout(60)
            server_socket.bind((host, port))
            server_socket.listen(1)
            print("서버가 시작되었습니다. 연결을 기다립니다...")
            conn, addr = server_socket.accept()
            print(f"{addr}와 연결되었습니다.")
            while True:
                data = conn.recv(1024).decode()
                if not data:
                    break
                print(f"받은 데이터: {data}")
                if data == "아포가토":
                    self.apocato()  # 아포가토 작업 시작
            conn.close()
        except Exception as e:
            print(f"서버 실행 중 오류 발생: {e}")
        finally:
            server_socket.close()

    def sub_motion_home(self):
        try:
            code = self._arm.set_position(*[229.6, -3.9, 461.1, -82.0, 88.7, -17.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[238.9, -7.0, 320.1, 128.4, 77.5, -165.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[211.0, 72.4, 326.1, 140.6, 76.7, -136.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[116.0, 64.7, 337.2, -140.3, 80.1, -65.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[157.6, 10.6, 329.9, -23.6, 88.2, -19.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[86.6, 100.5, 311.2, -56.5, 85.5, -5.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-16.3, 131.1, 313.0, -57.6, 82.5, 37.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-113.9, 71.1, 336.6, -64.2, 72.3, 80.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(2)
            code = self._arm.set_position(*[-148.2, -1.4, 317.0, -75.7, 81.7, 102.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def sub_motion_trash_capsule(self):
        try:
            code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed, mvacc=self._angle_acc, wait=False, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            time.sleep(1)
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)


# ================================= Robot Main Running code  ======================================= #
    # ======== original code ======== #
    def run(self):
        try:
            while self.is_alive:
                # Joint Motion
                if self.state == 'icecreaming':
                    # -------------- icecream start--------------------
                    try:
                        self.clientSocket.send('icecream_start'.encode('utf-8'))
                    except:
                        print('socket error')
                    time.sleep(int(self.order_msg['makeReq']['latency']))
                    self.motion_home()
                    # self.check_gripper()
                    while True:
                        if self.order_msg['makeReq']['latency'] in ['go', 'stop']:
                            break
                        time.sleep(0.2)
                    if self.order_msg['makeReq']['latency'] in ['go']:
                        self.motion_grab_capsule()
                        if self.order_msg['makeReq']['sealing'] in ['yes']:
                            self.motion_check_sealing()
                            try:
                                self.clientSocket.send('sealing_check'.encode('utf-8'))
                            except:
                                pass
                            count = 0
                            while True:
                                # if sealing_check request arrives or 5sec past
                                if self.order_msg['makeReq']['sealing'] in ['go', 'stop'] or count >= 5:
                                    print(self.order_msg['makeReq']['sealing'])
                                    break
                                time.sleep(0.2)
                                count += 0.2
                        if self.order_msg['makeReq']['sealing'] in ['go'] or self.order_msg['makeReq']['sealing'] not in ['yes', 'stop']:
                            #print('sealing_pass')
                            self.motion_place_capsule()
                            self.motion_grab_cup()
                            self.motion_topping()
                            self.motion_make_icecream()
                            self.motion_serve()
                            self.motion_trash_capsule()
                            self.motion_home()
                            print('icecream finish')
                            while True:
                                try:
                                    self.clientSocket.send('icecream_finish'.encode('utf-8'))
                                    break
                                except:
                                    time.sleep(0.2)
                                    print('socket_error')
                        else:
                            self.motion_place_fail_capsule()
                            self.motion_home()
                            self.clientSocket.send('icecream_cancel'.encode('utf-8'))
                            self.order_msg['makeReq']['sealing'] = ''
                    else:
                        while True:
                            try:
                                self.clientSocket.send('icecream_cancel'.encode('utf-8'))
                                break
                            except:
                                print('socket error')
                        self.order_msg['makeReq']['latency'] = 0
                    print('sendsendsendsnedasdhfaenbeijakwlbrsvz;ikbanwzis;fklnairskjf')
                    self.state = 'ready'

                elif self.state == 'test':
                    try:
                        self.clientSocket.send('test_start'.encode('utf-8'))
                    except:
                        print('socket error')
                    # self.motion_home()
                    # self.motion_grab_cup()
                    # self.motion_serve()

                elif self.state == 'greet':
                    self.motion_greet()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('greet_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    print('greet finish')
                    self.state = 'ready'

                elif self.state == 'dance_random':
                    dance_num = random.randrange(1, 4)
                    if dance_num == 1:
                        self.motion_dance_a()
                    elif dance_num == 2:
                        self.motion_dance_b()
                    elif dance_num == 3:
                        self.motion_dance_c()
                    while True:
                        try:
                            self.clientSocket.send('dance_random_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'dance_a':
                    self.motion_dance_a()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('dance_a_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'dance_b':
                    self.motion_dance_b()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('dance_b_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'dance_c':
                    self.motion_dance_c()
                    self.motion_home()
                    # self.clientSocket.send('dance_c_finish'.encode('utf-8'))
                    self.state = 'ready'

                elif self.state == 'breath':
                    try:
                        self.clientSocket.send('breath_start'.encode('utf-8'))
                        time.sleep(5)
                        self.clientSocket.send('breath_finish'.encode('utf-8'))
                    except:
                        print('socket error')

                elif self.state == 'sleep':
                    self.motion_sleep()
                    self.motion_home()
                    while True:
                        try:
                            self.clientSocket.send('sleep_finish'.encode('utf-8'))
                            break
                        except:
                            print('socket error')
                            time.sleep(0.2)
                    self.state = 'ready'

                elif self.state == 'comeon':
                    print('come_on start')
                    self.motion_come_on()
                    # self.motion_home()
                    self.state = 'ready'

                elif self.state == 'clean_mode':
                    try:
                        self.clientSocket.send('clean_mode_start'.encode('utf-8'))
                    except:
                        print('socket error')
                    self.state = 'ready'

                    code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self.state = 'ready'

                elif self.state == 'clean_mode_end':
                    code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
                    if not self._check_code(code, 'set_cgpio_digital'):
                        return
                    self.state = 'ready'


                elif self.state == 'ping': # robot_main.motion_make_icecream()
                    print('ping checked')
                    # self.motion_home()
                    self.state = 'ready'

                else:
                    pass

                # self.state = 'ready'
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
        self.alive = False
        
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)


        try:
            self.clientSocket.send('motion_dance_ff'.encode('utf-8'))
            print("Starting motion_dance_ff")
        except:
            print("Error in sending dance_ff_start")
            pass

        print("Starting motion_dance_ff")   
        self._angle_speed1 = 60
        self._angle_acc1 = 300
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed1,
                                            mvacc=self._angle_acc1, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            code = self._arm.set_servo_angle(angle=[212.0, -21.0, 112.0, 207.0, -0.8, 7.3], speed=self._angle_speed1,
                                                mvacc=self._angle_acc1, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[212.0, -38.0, 100.3, 180.4, -6.4, 6.0], speed=self._angle_speed1,
                                                mvacc=self._angle_acc1, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return


        self._angle_speed2 = 60
        self._angle_acc2 = 200
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed2,
                                            mvacc=self._angle_acc2, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

# ============================== final function tunning : banana, choco, strawberry apogato ============================== #
    def run_banana(self):
        start_a = False
        try:
            if not start_a:
                self.speak("바나나 아이스크림 주문 받았습니다.")
                self.motion_home()
                self.motion_grab_capsule_a()
                self.motion_check_sealing()
                self.motion_place_capsule()
                self.motion_grab_cup_a()
                self.motion_topping_a()
                self.motion_make_icecream_a() 
                self.motion_serve_a()
                self.motion_trash_capsule()
                self.motion_home()
                print('icecream finish')
                start_a = True
                self.speak("주문하신 아이스크림이 완성되었습니다")
                    
            else:
                self.motion_place_fail_capsule_a()
                self.motion_home()

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
        # ==================== error check ==================== #      
        
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)

    def run_choco(self):
        # -------------- icecream start--------------------
        start_b = False
        try:
            if not start_b:
                self.speak("초코 아이스크림 주문 받았습니다.")
                self.motion_home()
                self.motion_grab_capsule_b()
                self.motion_check_sealing()
                self.motion_place_capsule()
                self.motion_grab_cup_b()
                self.motion_topping_b()
                self.motion_make_icecream_b()
                self.motion_serve_b()
                self.motion_trash_capsule()
                self.motion_home()
                print('icecream finish')
                start_b = True
                self.speak("주문하신 아이스크림이 완성되었습니다")
            else:
                self.motion_place_fail_capsule_b()
                self.motion_home()

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
        # ==================== error check ==================== #      
        
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)

    def run_strawberry(self):
        # -------------- icecream start--------------------
        start_c = False
        try:
            if not start_c:
                self.speak("딸기 아이스크림 주문 받았습니다.")
                self.motion_home()
                self.motion_grab_capsule_c()
                self.motion_check_sealing()
                self.motion_place_capsule()
                self.motion_grab_cup_c()
                self.motion_topping_c()
                self.motion_make_icecream_c()
                self.motion_serve_c()
                self.motion_trash_capsule()
                self.motion_home()
                print('icecream finish')
                start_c = True
                self.speak("주문하신 아이스크림이 완성되었습니다")
            else:
                self.motion_place_fail_capsule_c()
                self.motion_home()

        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
        # ==================== error check ==================== #      
        
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)

    def apocato(self):
         # -------------- apocato start--------------------
        start_d = False
        try:
            if not start_d:
                self.speak("아포가토 아이스크림 주문 받았습니다.")
                print('icecream start')
                self.motion_home()
                self.motion_grab_capsule_a()
                self.motion_check_sealing()
                self.motion_place_capsule()
                self.motion_grab_cup_a()
                self.motion_topping_a()
                self.motion_make_icecream_a()
                self.motion_serve_a()
                self.motion_trash_capsule()
                self.motion_home()
                print('icecream finish')
                
                time.sleep(3)
                print('apogato start')
                self.apocato_make()
                self.motion_home()
                print('apogato finish')
                start_d = True
                self.speak("주문하신 아이스크림이 완성되었습니다")
            else:
                self.motion_place_fail_capsule_a()
                self.motion_home()
                
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
        # ==================== error check ==================== #      
        
        self.alive = False
        self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.release_state_changed_callback(self._state_changed_callback)
        
        if hasattr(self._arm, 'release_count_changed_callback'):
            self._arm.release_count_changed_callback(self._count_changed_callback)

# ============================== final function1 : hand gesture robot arm control ============================== #
    def final_hand_machine(self):
        while self.cap.isOpened(): # 비디오 캡처 객체가 열려 있는 동안 루프 실행
            ret, img = self.cap.read() # 프레임 읽기
            if not ret: # 프레임을 읽지 못하면 루프 탈출
                break
            
            img = cv2.flip(img, 1)  # 이미지를 좌우 반전
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # BGR에서 RGB로 색상 공간 변환
            result = self.hands.process(img) # 이미지를 손 추적 네트워크에 입력하여 결과 받기
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR) # 다시 RGB에서 BGR로 색상 공간 변환

            if result.multi_hand_landmarks is not None: # 손의 랜드마크가 있으면
                for res in result.multi_hand_landmarks: # 각 손에 대해
                    joint = np.zeros((21, 4)) # 21개의 랜드마크를 저장할 배열 생성
                    for j, lm in enumerate(res.landmark): # 각 랜드마크에 대해
                        joint[j] = [lm.x, lm.y, lm.z, lm.visibility] # x, y, z, visibility 값 저장

                    # Compute angles between joints
                    v1 = joint[[0,1,2,3,0,5,6,7,0,9,10,11,0,13,14,15,0,17,18,19], :3] # Parent joint
                    v2 = joint[[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20], :3] # Child joint
                    v = v2 - v1 # [20, 3]
                    # Normalize v
                    v = v / np.linalg.norm(v, axis=1)[:, np.newaxis] # 벡터 정규화


                    # Get angle using arcos of dot product (내적의 arcos로 각도 구하기)
                    angle = np.arccos(np.einsum('nt,nt->n',
                        v[[0,1,2,4,5,6,8,9,10,12,13,14,16,17,18],:],
                        v[[1,2,3,5,6,7,9,10,11,13,14,15,17,18,19],:])) # [15,]

                    angle = np.degrees(angle) # Convert radian to degree

                    d = np.concatenate([joint.flatten(), angle]) # 랜드마크와 각도 연결

                    self.seq.append(d) # 시퀀스에 추가

                    self.mp_drawing.draw_landmarks(img, res, self.mp_hands.HAND_CONNECTIONS) # 이미지에 랜드마크 그리기

                    if len(self.seq) < self.seq_length:  # 시퀀스 길이가 충분하지 않으면 계속
                        continue

                    input_data = np.expand_dims(np.array(self.seq[-self.seq_length:], dtype=np.float32), axis=0) # 입력 데이터 준비

                    y_pred = self.model.predict(input_data).squeeze() # 모델 예측

                    i_pred = int(np.argmax(y_pred)) # 가장 높은 확률의 인덱스
                    conf = y_pred[i_pred] # 해당 인덱스의 확률

                    if conf < 0.93: # 확률이 0.9보다 낮으면 계속
                        continue

                    self.action = self.actions[i_pred] # 예측된 액션
                    self.action_seq.append(self.action) # 액션 시퀀스에 추가

                    if len(self.action_seq) < 10: # 액션 시퀀스 길이가 3보다 작으면 계속
                        continue

                    self.this_action = '?' # 기본 값 설정
                    if self.action_seq[-1] == self.action_seq[-2] == self.action_seq[-3]== self.action_seq[-4]== self.action_seq[-5]== self.action_seq[-6]== self.action_seq[-7]== self.action_seq[-8]== self.action_seq[-9]== self.action_seq[-10]:
                        # if self.action_seq[-1] == self.action_seq[-2] == self.action_seq[-3]: # 마지막 3개의 액션이 같으면
                        self.this_action = self.action # 해당 액션 설정

                    cv2.putText(img, f'{self.this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)

            else: # 손의 랜드마크가 없으면
                err = "No Hand!!!!!!!!" # 에러 메시지 설정
                cv2.putText(img,err,(200,200),cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255),2) # 에러 메시지 출력
                self.this_action = '?' # 액션 초기화
                self.action_seq.clear() # 액션 시퀀스 초기화
                self.seq.clear() # 시퀀스 초기화

            if self.this_action != '?': # 액션이 '?'이 아니면
                if self.this_action == 'banana': # 액션이 'banana'이면
                    self.speak("바나나 아이스크림 주문 받았습니다.")
                    time.sleep(2)
                    playsound_thread = threading.Thread(target=self.play_audio, args=(self.audio_file,))
                    banana_final_thread = threading.Thread(target=self.run_banana)
                    
                    playsound_thread.start()
                    banana_final_thread.start()
                    
                    playsound_thread.join()
                    banana_final_thread.join()
                    time.sleep(2)
                    self.speak("주문하신 아이스크림이 완성되었습니다.")
                    break
        
                if self.this_action == 'choco': # 액션이 'choco'이면
                    self.speak("초코 아이스크림 주문 받았습니다.")
                    time.sleep(2)
                    playsound_thread = threading.Thread(target=self.play_audio, args=(self.audio_file,))
                    choco_final_thread = threading.Thread(target=self.run_choco)
                    
                    playsound_thread.start()
                    choco_final_thread.start()
                    
                    playsound_thread.join()
                    choco_final_thread.join()
                    
                    time.sleep(2)
                    self.speak("주문하신 아이스크림이 완성되었습니다")
                    break

                if self.this_action == 'strawberry':  # 액션이 'strawberry'이면
                    self.speak("딸기 아이스크림 주문 받았습니다.")
                    time.sleep(2)
                    playsound_thread = threading.Thread(target=self.play_audio, args=(self.audio_file,))
                    strawberry_final_thread = threading.Thread(target=self.run_strawberry)
                    
                    playsound_thread.start()
                    strawberry_final_thread.start()
                    
                    playsound_thread.join()
                    strawberry_final_thread.join()
                    
                    time.sleep(2)
                    self.speak("주문하신 아이스크림이 완성되었습니다")
                    break
                    
                self.this_action = '?' # 액션 초기화
                self.action_seq.clear()  # 액션 시퀀스 초기화
                self.seq.clear() # 시퀀스 초기화
            
            cv2.imshow('img', img) # 이미지 출력
            
            
            if cv2.waitKey(1) == ord('q'): # 'q' 키를 누르면 루프 탈출
                break

# ============================== final function2 : speech recognition robot arm control ============================== #

    def apogato_bluetooth(self):
        cap = cv2.VideoCapture(0)

        if not cap.isOpened():
            print("Error: Could not open camera.")
            return
        ret, frame = cap.read()
        server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)

        port = 1
        server_sock.bind(("", port))
        server_sock.listen(1)

        client_sock, address = server_sock.accept()
        print("Accepted connection from", address)

        try:
            while True:
                data = client_sock.recv(1024)
                if not data:
                    break
                print(f"받은 데이터: {data.decode()}")
                if data.decode() == "아포가토":
                    self.speak("아포가토 아이스크림 주문 받았습니다.")
                    time.sleep(2)
                    playsound_thread = threading.Thread(target=self.play_audio, args=(self.audio_file,))
                    apocato_final_thread = threading.Thread(target=self.apocato)
                    
                    playsound_thread.start()
                    apocato_final_thread.start()
                    
                    playsound_thread.join()
                    apocato_final_thread.join()
                    
                    self.speak("주문하신 아이스크림이 완성되었습니다")
        except Exception as e:
            print("에러 발생!!!!!!!!", e)
            pass
        
        print("연결 종료")
        client_sock.close()
        server_sock.close()

# ============================== final function3 : making a half taste icecream  ============================== #

    def final_run_half(self):
        
        self.speak("반반 아이스크림 주문 받았습니다.")
        time.sleep(2)
        # self.semi_final_run_half()
        playsound_thread = threading.Thread(target=self.play_audio, args=(self.audio_file,))
        semi_final_thread = threading.Thread(target=self.semi_final_run_half)
        
        playsound_thread.start()
        semi_final_thread.start()
        
        playsound_thread.join()
        semi_final_thread.join()
        
        self.speak("주문하신 아이스크림이 완성되었습니다")

    def semi_final_run_half(self):
        self.run_half_part1()
        self.run_half_part2_1()
        self.run_half_part2_2()
        self.run_half_part2_3()

    def run_half_part2_1(self):
        # -------------- icecream start--------------------
        try:
            if self.start == 0:
                # ####################  second 2_1 icecream #################### 
                self.motion_home()
                time.sleep(2)
                # self.motion_grab_capsule_b()
                # time.sleep(2)
                self.motion_grab_capsule_b_down()
                time.sleep(2)
                self.motion_check_sealing()
                time.sleep(2)
                self.motion_place_capsule()
                time.sleep(2)
                self.sub_motion_home()
                time.sleep(2)
                self.motion_home()
                time.sleep(2)
            else:
                self.motion_place_fail_capsule_a()
                self.motion_home()


                    
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
        # ==================== error check ==================== #      
        
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def run_half_part2_2(self):
        # -------------- icecream start--------------------
        a = 1
        try:
            if a == 1:
                ####################  second 2_2 icecream #################### 
                self.motion_home()
                self.motion_half_grap_cup_a()

                self.motion_home()
                self.motion_half_grap_cup_c()
                
                self.motion_home()
                self.motion_serve_c_x()

                a = 0
        
            else:
                self.motion_place_fail_capsule_a()
                self.motion_home()
                a = 0

                    
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
            
            # ==================== error check ==================== #      
            
            # self.alive = False
            # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
            # self._arm.release_state_changed_callback(self._state_changed_callback)
            
            # if hasattr(self._arm, 'release_count_changed_callback'):
            #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def run_half_part2_3(self):
        # -------------- icecream start--------------------
        a = 1
        try:
            if a == 1:
                ####################  second icecream ####################
                self.motion_trash_capsule()
                self.motion_home()
                a = 0
        
            else:
                self.motion_place_fail_capsule_a()
                self.motion_home()
                a = 0

                    
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        
        # ==================== error check ==================== #      
        
        # self.alive = False
        # self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)
        # self._arm.release_state_changed_callback(self._state_changed_callback)
        
        # if hasattr(self._arm, 'release_count_changed_callback'):
        #     self._arm.release_count_changed_callback(self._count_changed_callback)

    def run_half_part1(self):
        try:
            ###################  first icecream #################### 
            # self.motion_home()
            # self.motion_grab_capsule_a()
            # self.motion_check_sealing()
            # self.motion_place_capsule()

            # self.motion_grab_cup_a()
            # self.motion_topping_a()

            # self.motion_serve_a()
            # self.motion_home()
            # self.motion_grab_capsule_x()
            # self.motion_place_capsule() ###
            # self.motion_grab_cup_a()
            # self.motion_topping_a()

            # self.motion_serve_c()
            # self.motion_trash_capsule()
            # self.motion_home()
            
            
            
            self.motion_home()
            self.motion_grab_capsule_b()
            self.motion_check_sealing()
            self.motion_place_capsule()

            self.motion_grab_cup_b()
            self.motion_topping_b()

            self.motion_serve_b()
            self.motion_home()
            self.motion_grab_capsule_x()
            self.motion_place_capsule() ###
            self.motion_grab_cup_b()
            self.motion_topping_b()

            self.motion_serve_c()
            self.motion_trash_capsule()
            self.motion_home()
            
            

            
        except Exception as e:
            self.pprint('MainException: {}'.format(e))
        

    # ============================== final function4 : hand robot arm tracking ============================== #

    def run_robot_arm_tracking(self):
        self.calibration()
        self.tracking_home()
        self.speak("손 추적 시스템 작동 시작하겠습니다")
        audio_thread = threading.Thread(target=self.play_audio_game, args=(self.game_file,))
        adsd_thread = threading.Thread(target=self.hand_camera)
        
        audio_thread.start()
        adsd_thread.start()
        
        adsd_thread.join()
        
        self.stop_event.set()
        audio_thread.join()
        
        # self.adsd()  # 손 추적 시작
        time.sleep(1)
        self.speak("손 추적 시스템 종료하겠습니다")
        
    
    def dh_transform(self, theta, d, a, alpha):
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        # print(f" dh_transform theta, alpha : {theta,alpha}")
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def forward_kinematics(self, dh_params, thetas):
        T = np.eye(4)
        for i, params in enumerate(dh_params):
            theta = params[0] + thetas[i]
            d = params[1]
            a = params[2]
            alpha = params[3]
            T_i = self.dh_transform(theta, d, a, alpha)
            T = np.dot(T, T_i)
            # print(f"forward_kinematics T : {T}")
        return T
    
    def inverse_kinematics(self, target_pos, target_orientation, dh_params, initial_guess):
        def objective(thetas):
            T = self.forward_kinematics(dh_params, thetas)
            pos = T[:3, 3]
            ori = T[:3, :3]
            pos_error = np.linalg.norm(pos - target_pos)
            ori_error = np.linalg.norm(ori - target_orientation)
            # print(f"ori_error : {ori_error}")
            return pos_error + ori_error
        bounds = [(-360, 360), (-150, 150), (-3.5, 300), (-360, 360), (-124, 124), (-360, 360)]
        result = minimize(objective, initial_guess, bounds=bounds)
        # print(f" inverse_kinematics result : {result}")
        return result.x
    
    def process_input(self, x_input, y_input, z_input):
        target_pos = np.array([x_input, y_input, z_input])
        target_orientation = np.eye(3)  # 엔드 이펙터가 지면과 평행하게 유지
        initial_guesses = [
            self.current_state,
            np.random.uniform(-180, 180, 6),
            np.random.uniform(-180, 180, 6),
            np.random.uniform(-180, 180, 6),
        ]
        solutions = []
        for initial_guess in initial_guesses:
            thetas = self.inverse_kinematics(target_pos, target_orientation, self.dh_params, initial_guess)
            solutions.append(thetas)
        min_move_solution = min(solutions, key=lambda thetas: np.linalg.norm(thetas - self.current_state))
        self.current_state = min_move_solution
        thetas_deg = self.current_state
        self.solutions.append(thetas_deg)
        # print(f"process_input : {thetas_deg}")
        # print(f"Move joint angles (in degrees): theta1={thetas_deg[0]:.2f}, theta2={thetas_deg[1]:.2f}, theta3={thetas_deg[2]:.2f}, theta4={thetas_deg[3]:.2f}, theta5={thetas_deg[4]:.2f}, theta6={thetas_deg[5]:.2f}")
        code = self._arm.set_servo_angle(angle=thetas_deg, speed=20, mvacc=500, wait=False)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # print('code', code)
        
    def tracking_home(self):
        code = self._arm.set_servo_angle(angle=[269.4, -34, 18.9, 182.6, 38.1, 0.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        
    def calibration(self):
        cv_file = cv2.FileStorage("calibration_params_final2.xml", cv2.FILE_STORAGE_READ)
        self.camera_matrix = cv_file.getNode("cameraMatrixL").mat()
        self.new_camera_matrix = cv_file.getNode("newCameraMatrixL").mat()
        self.dist_coeffs = cv_file.getNode("distL").mat()
        self.roi = cv_file.getNode("roiL").mat()
        cv_file.release()
        # print("Camera Matrix:")
        # print(self.camera_matrix)
        # print("New Camera Matrix:")
        # print(self.new_camera_matrix)
        # print("Distortion Coefficients:")
        # print(self.dist_coeffs)
        # print("ROI:")
        # print(self.roi)
        
    def pixel_to_world(self, pixel_x, pixel_y):
        real_x_cali_mm = -3.6+ ((pixel_x-356) * 1.7) #변화량
        real_z_cali_mm = 300 + ((pixel_y-227) *(-1.2))
        print(f"Pixel coordinates: ({pixel_x}, {pixel_y})")
        # print(f"Real world coordinates: ({real_x_mm:.2f} mm, {real_y_mm:.2f} mm)")
        print(f"Real world calibration coordinates: ({real_x_cali_mm:.2f} mm, {real_z_cali_mm:.2f} mm)")
        return real_x_cali_mm, real_z_cali_mm
    
    # def hand_camera(self):
    #     time.sleep(0.5)
    #     self.start_cleanup_timer()
    #     prev_x, prev_y = None, None
    #     canvas = None
    #     last_time = time.time()
    #     initial_hand_position = None
    #     tracking_margin = 75
    #     self.start_cleanup_timer_exit()
        
    #     with self.mp_hands.Hands(max_num_hands=1) as hands:
    #         while self.cap.isOpened():
    #             ret, frame = self.cap.read()
    #             if not ret:
    #                 break
                
    #             frame = cv2.flip(frame, 1)
    #             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #             result = hands.process(frame_rgb)
    #             current_time = time.time()
                
    #             if self.roi_defined:
    #                 cv2.rectangle(frame, self.roi_start, self.roi_end, (0, 255, 0), 2)
    #                 cv2.putText(frame, f'Start: {self.roi_start}', (self.roi_start[0], self.roi_start[1] - 10),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    #                 cv2.putText(frame, f'End: {self.roi_end}', (self.roi_end[0], self.roi_end[1] + 20),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                
    #             # if self.roi_defined and not self.hand_tracked:
    #             #             if (self.roi_start[0] <= x <= self.roi_end[0]) and (self.roi_start[1] <= y <= self.roi_end[1]):
    #             #                 self.hand_tracked = True  # 손이 인식되었음을 표시
    #             #                 initial_hand_position = (x, y)
    #             #                 print(f'Hand initially detected at: ({x}, {y})')
    #             #         self.hand_tracked =True
    #             #         if self.hand_tracked:
                
                
    #             if result.multi_hand_landmarks:
    #                 for hand_landmarks in result.multi_hand_landmarks:
    #                     x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x * frame.shape[1])
    #                     y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y * frame.shape[0])
    #                     if (self.roi_start[0] <= x <= self.roi_end[0]) and (self.roi_start[1] <= y <= self.roi_end[1]):
    #                         initial_hand_position = (x, y)
    #                         if initial_hand_position is None:
    #                             initial_hand_position = (x, y)
    #                         if initial_hand_position is not None:
    #                             if prev_x is None or prev_y is None:
    #                                 prev_x, prev_y = x, y
    #                             if abs(x - prev_x) < tracking_margin and abs(y - prev_y) < tracking_margin:
    #                                 # if canvas is None:
    #                                 #     canvas = frame.copy()
    #                                 if current_time - last_time >= 1:
                                        
    #                                     point_homogeneous = np.array([x, y, 1.0], dtype=np.float32).reshape(3, 1)
    #                                     transformed_point = np.dot(self.H, point_homogeneous)
    #                                     transformed_point /= transformed_point[2]
    #                                     transformed_coordinate = tuple(transformed_point[:2].flatten())
                                        
    #                                     if 0 <= transformed_coordinate[0] < self.img_width and 0 <= transformed_coordinate[1] < self.img_height:
    #                                         self.coordinates.append(transformed_coordinate)
    #                                     else:
    #                                         print(f"Transformed coordinate {transformed_coordinate} is out of bounds.")
    #                                     last_time = current_time
    #                                 prev_x, prev_y = x, y
    #                                 self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
    #                 cv2.imshow('Hand Tracking', frame)
    #                 if cv2.waitKey(1) & 0xFF == ord('q'):
    #                     self.cleanup_timer.cancel()
    #                     break
    #                 # 주기적으로 좌표를 처리
    #                 if current_time - self.last_processed_time > self.process_interval:
    #                     self.process_latest_coordinates()
    #                     self.last_processed_time = current_time
    #         self.cleanup_and_exit()





# ===================== original ================================= #
    # def hand_camera(self):
    #     time.sleep(0.5)
    #     self.start_cleanup_timer()
    #     prev_x, prev_y = None, None
    #     canvas = None
    #     last_time = time.time()
    #     tracking_margin = 75  # 이 값을 조정하거나 동적으로 변경할 수 있습니다
    #     self.start_cleanup_timer_exit()

    #     with self.mp_hands.Hands(max_num_hands=1) as hands:
    #         while self.cap.isOpened():
    #             ret, frame = self.cap.read()
    #             if not ret:
    #                 break

    #             frame = cv2.flip(frame, 1)
    #             frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #             result = hands.process(frame_rgb)
    #             current_time = time.time()

    #             if self.roi_defined:
    #                 cv2.rectangle(frame, self.roi_start, self.roi_end, (0, 255, 0), 2)
    #                 cv2.putText(frame, f'Start: {self.roi_start}', (self.roi_start[0], self.roi_start[1] - 10),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    #                 cv2.putText(frame, f'End: {self.roi_end}', (self.roi_end[0], self.roi_end[1] + 20),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

    #             hand_detected = False
    #             if result.multi_hand_landmarks:
    #                 for hand_landmarks in result.multi_hand_landmarks:
    #                     x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x * frame.shape[1])
    #                     y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y * frame.shape[0])
                        
    #                     # ROI 안에 손이 있는지 확인
    #                     if (self.roi_start[0] <= x <= self.roi_end[0]) and (self.roi_start[1] <= y <= self.roi_end[1]):
    #                         hand_detected = True
    #                         if prev_x is None or prev_y is None:
    #                             prev_x, prev_y = x, y
    #                         if abs(x - prev_x) < tracking_margin and abs(y - prev_y) < tracking_margin:
    #                             if current_time - last_time >= 1:
    #                                 point_homogeneous = np.array([x, y, 1.0], dtype=np.float32).reshape(3, 1)
    #                                 transformed_point = np.dot(self.H, point_homogeneous)
    #                                 transformed_point /= transformed_point[2]
    #                                 transformed_coordinate = tuple(transformed_point[:2].flatten())
                                    
    #                                 if 0 <= transformed_coordinate[0] < self.img_width and 0 <= transformed_coordinate[1] < self.img_height:
    #                                     self.coordinates.append(transformed_coordinate)
    #                                 else:
    #                                     print(f"Transformed coordinate {transformed_coordinate} is out of bounds.")
    #                                 last_time = current_time
    #                             prev_x, prev_y = x, y
    #                             self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                
    #             if not hand_detected:
    #                 # 손이 인식되지 않았을 때의 처리
    #                 if prev_x is not None and prev_y is not None:
    #                     print("손을 인식하지 못했습니다. 다시 시도 중...")
    #                     # 손 인식을 다시 시도할 수 있도록 초기화하거나 처리 로직 추가
    #                     prev_x, prev_y = None, None  

    #             cv2.imshow('Hand Tracking', frame)
    #             if cv2.waitKey(1) & 0xFF == ord('q'):
    #                 self.cleanup_timer.cancel()
    #                 break

    #             # 주기적으로 좌표를 처리
    #             if current_time - self.last_processed_time > self.process_interval:
    #                 self.process_latest_coordinates()
    #                 self.last_processed_time = current_time

    #         self.cleanup_and_exit()

    # 강제 종료시 마지막에 좌표가 튀는 현상이 있음, 20초 뒤에 꺼지게 해여하는데 안꺼짐
    # def process_latest_coordinates(self):
    #     if self.coordinates:
    #         pixel_x, pixel_y = self.coordinates[-1]
    #         self.real_x_cali_mm, self.real_z_cali_mm = self.pixel_to_world(pixel_x, pixel_y)
    #         self.process_input(self.real_x_cali_mm, -300, self.real_z_cali_mm)
            
    # def start_cleanup_timer(self):
    #     # self.cleanup_timer = Timer(self.cleanup_interval, self.cleanup_lists)
    #     # self.cleanup_timer.start()
    #     # print("start_cleanup_timer")
    #     if self.cleanup_timer is not None:
    #         self.cleanup_timer.cancel()  # 기존 타이머 취소
    #     self.cleanup_timer = Timer(self.cleanup_interval, self.cleanup_lists)
    #     self.cleanup_timer.start()
    #     print("start_cleanup_timer")

    # def cleanup_lists(self):
    #     # if len(self.coordinates) > 0:
    #     #     del self.coordinates[:len(self.coordinates) // 2]
    #     # if len(self.solutions) > 0:
    #     #     del self.solutions[:len(self.solutions) // 2]
    #     # self.start_cleanup_timer()  # 타이머를 다시 시작
    #     # print("cleanup_lists")
    #     if len(self.coordinates) > 0:
    #         del self.coordinates[:len(self.coordinates) // 2]
    #     if len(self.solutions) > 0:
    #         del self.solutions[:len(self.solutions) // 2]
    # # cleanup_lists 호출 후, 타이머를 다시 설정하지 않음
    #     print("cleanup_lists")

    # def start_cleanup_timer_exit(self):
    #     # self.cleanup_timer_exit = threading.Timer(20.0, self.cleanup_and_exit)
    #     # self.cleanup_timer_exit.start()
    #     if self.cleanup_timer_exit is not None:
    #         self.cleanup_timer_exit.cancel()  # 기존 타이머 취소
    #     self.cleanup_timer_exit = threading.Timer(1000.0, self.cleanup_and_exit)
    #     self.cleanup_timer_exit.start()

    # def cleanup_and_exit(self):
    #     self.cap.release()
    #     cv2.destroyAllWindows()
    
    
    def hand_camera(self):
        def run():
            time.sleep(0.5)
            self.start_cleanup_timer()
            prev_x, prev_y = None, None
            last_time = time.time()
            tracking_margin = 75
            self.start_cleanup_timer_exit()

            with self.mp_hands.Hands(max_num_hands=1) as hands:
                while self.cap.isOpened() and not self.exit_flag:
                    ret, frame = self.cap.read()
                    if not ret:
                        break

                    frame = cv2.flip(frame, 1)
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    result = hands.process(frame_rgb)
                    current_time = time.time()

                    if self.roi_defined:
                        cv2.rectangle(frame, self.roi_start, self.roi_end, (0, 255, 0), 2)
                        cv2.putText(frame, f'Start: {self.roi_start}', (self.roi_start[0], self.roi_start[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
                        cv2.putText(frame, f'End: {self.roi_end}', (self.roi_end[0], self.roi_end[1] + 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                    hand_detected = False
                    if result.multi_hand_landmarks:
                        for hand_landmarks in result.multi_hand_landmarks:
                            x = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].x * frame.shape[1])
                            y = int(hand_landmarks.landmark[self.mp_hands.HandLandmark.WRIST].y * frame.shape[0])
                            
                            if (self.roi_start[0] <= x <= self.roi_end[0]) and (self.roi_start[1] <= y <= self.roi_end[1]):
                                hand_detected = True
                                if prev_x is None or prev_y is None:
                                    prev_x, prev_y = x, y
                                if abs(x - prev_x) < tracking_margin and abs(y - prev_y) < tracking_margin:
                                    if current_time - last_time >= 1:
                                        point_homogeneous = np.array([x, y, 1.0], dtype=np.float32).reshape(3, 1)
                                        transformed_point = np.dot(self.H, point_homogeneous)
                                        transformed_point /= transformed_point[2]
                                        transformed_coordinate = tuple(transformed_point[:2].flatten())
                                        
                                        if 0 <= transformed_coordinate[0] < self.img_width and 0 <= transformed_coordinate[1] < self.img_height:
                                            self.coordinates.append(transformed_coordinate)
                                        else:
                                            print(f"Transformed coordinate {transformed_coordinate} is out of bounds.")
                                        last_time = current_time
                                    prev_x, prev_y = x, y
                                    self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
                    
                    if not hand_detected:
                        if prev_x is not None and prev_y is not None:
                            print("손을 인식하지 못했습니다. 다시 시도 중...")
                            prev_x, prev_y = None, None  

                    cv2.imshow('Hand Tracking', frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        self.cleanup_timer.cancel()
                        self.exit_flag = True
                        break

                    if current_time - self.last_processed_time > self.process_interval:
                        self.process_latest_coordinates()
                        self.last_processed_time = current_time

                self.cleanup_and_exit()

        thread = threading.Thread(target=run)
        thread.start()
        timer = threading.Timer(20.0, self.set_exit_flag)
        timer.start()

        # `thread.join()`이 타이머 설정 이후로 이동하여 조건을 충족하는지 확인합니다.
        timer.join()
        thread.join()

    def set_exit_flag(self):
        self.exit_flag = True
        print("Function timed out and will be terminated.")

    def process_latest_coordinates(self):
        if self.coordinates:
            pixel_x, pixel_y = self.coordinates[-1]
            self.real_x_cali_mm, self.real_z_cali_mm = self.pixel_to_world(pixel_x, pixel_y)
            self.process_input(self.real_x_cali_mm, -300, self.real_z_cali_mm)
            
    def start_cleanup_timer(self):
        if self.cleanup_timer is not None:
            self.cleanup_timer.cancel()  # 기존 타이머 취소
        self.cleanup_timer = Timer(self.cleanup_interval, self.cleanup_lists)
        self.cleanup_timer.start()
        print("start_cleanup_timer")

    def cleanup_lists(self):
        if len(self.coordinates) > 0:
            del self.coordinates[:len(self.coordinates) // 2]
        if len(self.solutions) > 0:
            del self.solutions[:len(self.solutions) // 2]
        print("cleanup_lists")

    def start_cleanup_timer_exit(self):
        if self.cleanup_timer_exit is not None:
            self.cleanup_timer_exit.cancel()  # 기존 타이머 취소
        self.cleanup_timer_exit = Timer(20.0, self.cleanup_and_exit)
        self.cleanup_timer_exit.start()

    def cleanup_and_exit(self):
        self.cap.release()
        cv2.destroyAllWindows()


    # ============================== final function5 : trash ============================== #
    
    def tracking_a_banana(self):
        code = self._arm.set_position(*[-180.5, -75.9, 275.7, -78.9, 79.9, 127.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-171.6, -95.5, 217.8, -166.9, 80.3, 52.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-191.0, -113.9, 208.2, 175.1, 79.2, 42.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-227.0, -135.6, 199.6, 175.7, 79.5, 48.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(4)
        code = self._arm.set_position(*[-214.0, -132.8, 344.0, -61.5, 82.5, 168.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-14.9, -218.3, 333.5, 57.4, 83.0, -36.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        self.tracking_home()

    def tracking_b_choco(self):
        code = self._arm.set_position(*[-164.7, -48.6, 213.8, 178.2, 76.6, 59.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-136.8, -57.5, 198.0, 124.1, 83.6, 27.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-133.5, -99.0, 195.3, 176.2, 83.8, 77.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-129.4, -122.4, 194.4, -176.5, 83.0, 78.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-130.0, -122.7, 202.3, 75.4, 88.4, -29.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(4)
        code = self._arm.set_position(*[-155.8, -133.7, 284.0, -56.4, 85.3, -165.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-67.9, -199.1, 329.8, 32.4, 78.3, -66.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-9.4, -196.3, 344.8, 78.7, 84.7, -15.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        self.tracking_home()

    def tracking_c_strawberry(self):
        code = self._arm.set_position(*[-141.4, 10.7, 212.0, -150.4, 81.8, 70.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-139.7, 8.7, 211.9, -161.9, 82.4, 61.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-123.5, -30.0, 211.0, -171.7, 81.9, 79.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-75.2, -92.1, 210.7, 167.9, 81.5, 102.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-74.7, -96.2, 206.9, 176.3, 76.3, 110.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-54.3, -131.9, 208.3, 176.4, 77.7, 107.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-54.7, -133.5, 221.1, 95.0, 87.4, 25.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(4)
        code = self._arm.set_position(*[-63.2, -137.3, 276.3, -14.4, 84.7, -94.7], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-17.4, -176.7, 313.4, 177.9, 89.2, 85.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        self.tracking_home()
    
    def dh_transform_trash(self, theta, d, a, alpha):
        theta = np.deg2rad(theta)
        alpha = np.deg2rad(alpha)
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])
        
    def forward_kinematics_trash(self, dh_params, thetas):
        T = np.eye(4)
        for i, params in enumerate(dh_params[:-1]):
            theta = params[0] + thetas[i]
            d = params[1]
            a = params[2]
            alpha = params[3]
            T_i = self.dh_transform_trash(theta, d, a, alpha)
            T = np.dot(T, T_i)
        return T

    def inverse_kinematics_trash(self, target_pos, target_orientation, dh_params, initial_guess):
        def objective(thetas):
            T = self.forward_kinematics_trash(dh_params, thetas)
            pos = T[:3, 3]
            ori = T[:3, :3]
            pos_error = np.linalg.norm(pos - target_pos)
            ori_error = np.linalg.norm(ori - target_orientation)

            

            return pos_error + ori_error 
        bounds = [(-360, 360), (-150, 150), (-3.5, 300), (-360, 360), (-124, 124), (-360, 360)]
        result = minimize(objective, initial_guess, bounds=bounds)
        return result.x
    
    def process_input_trash(self, x_input, y_input, z_input):
    
        target_pos = np.array([x_input, y_input, z_input])
        target_orientation = np.array([
            [1, 0, 0],
            [0, 1, 0],
            [0, 0, 1]
        ]) 
        initial_guesses = [
            self.current_state,
            np.random.uniform(-180, 180, 6),
            np.random.uniform(-180, 180, 6),
            np.random.uniform(-180, 180, 6),
        ]
        solutions_trash = []
        for initial_guess in initial_guesses:
            thetas_trash = self.inverse_kinematics_trash(target_pos, target_orientation, self.dh_params, initial_guess)
            solutions_trash.append(thetas_trash)
        min_move_solution_trash = min(solutions_trash, key=lambda thetas_trash: np.linalg.norm(thetas_trash - self.current_state))
        self.current_state = min_move_solution_trash
        thetas_deg_trash = self.current_state
        self.solutions_trash.append(thetas_deg_trash)
        
        print(f"Move joint angles (in degrees): theta1={thetas_deg_trash[0]:.2f}, theta2={thetas_deg_trash[1]:.2f}, theta3={thetas_deg_trash[2]:.2f}, theta4={thetas_deg_trash[3]:.2f}, theta5={thetas_deg_trash[4]:.2f}, theta6={thetas_deg_trash[5]:.2f}")
        code = self._arm.set_servo_angle(angle=thetas_deg_trash, speed=20, mvacc=500, wait=False)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        
        code = self._arm.set_servo_angle(angle=[thetas_deg_trash[0],thetas_deg_trash[1],thetas_deg_trash[2],thetas_deg_trash[3],thetas_deg_trash[4]+self.offset_deg], speed=20, mvacc=500, wait=False)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)

    def motion_home_trash1(self):    ###### 1사분면에 쓰레기가 있는 경우 #########   
        try:
            print("1")
            code = self._arm.set_position(*[-104.5, -115.0, 313.6, -75.8, 85.7, 153.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-72.4, -197.3, 347.0, -38.6, 81.1, -151.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[129.2, -215.2, 331.9, -60.7, 78.1, -119.6], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[216.6, -127.4, 327.7, -73.2, 83.6, -103.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        except Exception as e:
            self.pprint('MainException: {}'.format(e))

    def motion_home_trash3(self):      
        code = self._arm.set_position(*[-163.6, 5.4, 288.4, -17.0, 87.7, 159.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[-94.1, 133.3, 287.8, 42.6, 87.0, 164.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[72.2, 143.7, 282.5, 75.3, 87.1, 136.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_position(*[158.3, 21.6, 280.3, -78.2, 87.8, -73.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)

    def trash(self,input_x,input_y):
        try:
            ################ home 근처 쓰레기 ###############################
            if input_x < -110 :
                print("Home위치 쓰레기 감지")
                self.motion_home()
                self.current_state=np.array([179.2, -42.1, 7.4, 186.7, 41.5, -1.6])
                if 0< input_y <50  :
                    self.offset_x=15                            ###self.offset_x,y 실험값
                    self.offset_y=-15
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90                   ###self.offset_deg 실험값
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif 50 <= input_y <= 150 :
                    self.offset_x=23
                    self.offset_y=-15
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif -13 <= input_y <=-0  :
                    self.offset_x=12.5
                    self.offset_y=-5
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif  -20 <= input_y <-13  :
                    self.offset_x=0
                    self.offset_y=0
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif  -50 <= input_y <-20  :
                    self.offset_x= 10
                    self.offset_y=-20
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                elif  input_y <-50  :   #####################okay#####################3
                    self.offset_x= 5
                    self.offset_y=-10
                    if -130 <= input_x <-110 :
                        self.offset_deg = -90
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                        
                    elif -160 <= input_x <-130 :
                        self.offset_deg = -87
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
                    
                    elif -200 <= input_x <-160 :
                        self.offset_deg = -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)

                    elif -300 <= input_x < -160 :
                        self.offset_deg= -84
                        self.process_input_trash(input_x+self.offset_x,input_y+5+ self.offset_y,331)
               
               
                time.sleep(10)
                code = self._arm.close_lite6_gripper()
                if not self._check_code(code, 'close_lite6_gripper'):
                    return
                time.sleep(5)
                self.motion_home()
                time.sleep(1)
                self.throw='Home'
                self.throw_trash()

            elif -110 <= input_x <=160 :
                print("That position is out of Boundary")

            
            elif 160 < input_x < 380:
                
                if input_y <= 0 :
                    ################ 1사분면 근처 쓰레기 ############################
                    print("1사분면 쓰레기 감지")
                    self.motion_home_trash1()
                    self.current_state = np.array([329.5,-7,33.6,179.6,51.2,-5.8])

                    if 160 < input_x < 270 :
                        self.offset_deg=-94
                        if input_x <225 :
                            self.offset_x= -20
                            self.offset_y=-10
                        elif 225<= input_x :
                            self.offset_x= -27.5
                            self.offset_y=-10

                        self.process_input_trash(input_x+self.offset_x,input_y+self.offset_y,332)

                    elif 270 <= input_x <380 :
                        self.offset_deg= -95
                        if input_x <300 :
                            self.offset_x= -27.5
                            self.offset_y=-10

                        elif 300< input_x <350 :
                            self.offset_x= -31
                            self.offset_y=-10

                        elif 350<= input_x :
                            self.offset_x= -35
                            self.offset_y=-10
                        self.process_input_trash(input_x+self.offset_x,input_y+self.offset_y,332)
                        

                    time.sleep(10)
                    code = self._arm.close_lite6_gripper()
                    if not self._check_code(code, 'close_lite6_gripper'):
                        return
                    time.sleep(4)
                    self.process_input(216.6,-127.4,327.7)
                    self.throw='trash1'
                    time.sleep(1)
                    self.throw_trash()

                elif input_y >0 :
                    ################ 3사분면 근처 쓰레기 ############################
                    print("3사분면 쓰레기 감지")
                    self.motion_home_trash3()
                    self.current_state =np.array([9.4, -31.9, 4, 185.1, 54.7, -5.1])
                    

                    if 160 < input_x < 270 :
                        self.offset_deg=-96
                        self.process_input_trash(input_x,input_y-3,332)

                    elif 270 <= input_x <380 :
                        self.offset_deg= -95
                        self.process_input_trash(input_x,input_y-3,332)

                    time.sleep(20)

                    code = self._arm.close_lite6_gripper()
                    if not self._check_code(code, 'close_lite6_gripper'):
                        return
                    time.sleep(4)
                    

                    
                    self.process_input(158.3,-21.6,280.3)
            
        except:
            self.motion_home()
            print("That position is out of Boundary")

    def throw_trash(self):
            if self.throw == 'Home':
                code = self._arm.set_position(*[-154.8, -7.0, 320.0, -37.6, 81.6, 142.9], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-124.0, 75.9, 320.4, -179.4, 86.5, -35.5], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-134.1, 100.9, 285.9, 107.6, 80.4, -119.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(3)
                code = self._arm.open_lite6_gripper()
                if not self._check_code(code, 'open_lite6_gripper'):
                    return
                time.sleep(3)
                code = self._arm.stop_lite6_gripper()
                if not self._check_code(code, 'stop_lite6_gripper'):
                    return
                time.sleep(1)
            
                code = self._arm.set_position(*[-159.6, 31.7, 294.3, 166.7, 83.5, -27.4], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)


            elif self.throw == 'trash1' :
                code = self._arm.set_position(*[216.6, -127.4, 327.7, -58.3, 83.4, -89.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[4.2, -244.0, 352.2, -26.2, 59.2, -112.1], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-231.5, -53.0, 362.1, -1.0, 51.5, -176.8], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-208.6, 1.8, 339.9, 51.2, 83.0, -141.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-176.8, 62.7, 321.8, 116.2, 80.8, -93.2], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[-128.4, 114.8, 279.0, 157.0, 63.7, -77.3], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(3)
                code = self._arm.open_lite6_gripper()
                if not self._check_code(code, 'open_lite6_gripper'):
                    return
                time.sleep(5)
                code = self._arm.stop_lite6_gripper()
                if not self._check_code(code, 'stop_lite6_gripper'):
                    return  
                time.sleep(1)
                code = self._arm.set_position(*[-174.7, -0.9, 311.2, -91.2, 86.0, 86.0], speed=self._tcp_speed, mvacc=self._tcp_acc, radius=0.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
            else:
                print("Not designed")

if __name__ == '__main__':
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.184', baud_checkset=False)
    robot_main = RobotMain(arm)
    print("start!!!!!!!!!!!!!!!!!!")
    # robot_main.run_robot_arm_tracking()
    
    robot_main.run_half_part1()
    # robot_main.tracking_a_banana()
    # robot_main.motion_grab_capsule_b_down()
    # robot_main.motion_greet()
    # robot_main.motion_grab_capsule_b_down()
    # robot_main.tracking_a_banana()
    # robot_main.start_robo()
    print("finish!!!!!!!!!!!!!!!!!")
    # robot_main.motion_home()





