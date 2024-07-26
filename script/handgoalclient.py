# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from hand_gesture_msgs.action import HandGestureAction
# import cv2
# from cv_bridge import CvBridge
# import pickle
# from ultralytics import YOLO
# import threading
# import time

# class HandGestureActionClient(Node):
#     def __init__(self):
#         super().__init__('hand_gesture_action_client')
#         self._action_client = ActionClient(self, HandGestureAction, 'hand_gesture')
        
#         # 카메라 초기화 및 프레임 캡처 스레드
#         self.cap = cv2.VideoCapture(0,cv2.CAP_V4L2)
#         self.frame = None
#         self.frame_lock = threading.Lock()
#         self.frame_counter = 0
#         self.sample_interval = 5  # 매 5번째 프레임마다 처리
#         self.stop_thread = False
#         self.camera_thread = threading.Thread(target=self.capture_frame)
#         self.camera_thread.start()
        
#         self.bridge = CvBridge()

#         self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
#         self.model.verbose = False
#         with open("/home/lee/Desktop/topic_test/calibration_data.pkl", "rb") as f:
#             calibration_data = pickle.load(f)
#             self.cameraMatrixL = calibration_data["cameraMatrixL"]
#             self.distL = calibration_data["distL"]

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.last_sent_time = time.time()
#         self.cooldown_period = 20  # 20초 대기 시간 설정
#         self.object_coords = []
#         self.last_detection_time = time.time()  # 마지막 물체 인식 시간 초기화
        
#     def capture_frame(self):
#         while not self.stop_thread:
#             if not self.cap.isOpened():
#                 self.get_logger().error('카메라를 열 수 없습니다.')
#                 break
#             ret, img = self.cap.read()
#             if not ret:
#                 self.get_logger().error('카메라에서 프레임을 캡처하지 못했습니다.')
#                 continue
#             with self.frame_lock:
#                 self.frame = img
#             time.sleep(0.1)  # Adjust sleep time if needed

#     def timer_callback(self):
#         current_time = time.time()

#         if self.frame is None:
#             return

#         with self.frame_lock:
#             self.frame_counter += 1
#             if self.frame_counter % self.sample_interval != 0:
#                 return  # Skip this frame based on the sampling interval
            
#             img = self.frame.copy()  # Process the current frame
        
#         # 인식된 물체 좌표 얻기
#         coordinates = self.detect_objects(img)
#         self.object_coords = coordinates

#         if coordinates:
#             self.last_detection_time = current_time  # 마지막 인식 시간 갱신
#             if current_time - self.last_sent_time >= self.cooldown_period:
#                 # 각 물체 좌표 전송
#                 x, y = self.object_coords[0]
#                 self.send_goal(x, y)
#                 self.last_sent_time = current_time  # 마지막 전송 시간 갱신
#         else:
#             # 물체 인식되지 않은 시간 계산
#             if current_time - self.last_detection_time >= 5:
#                 self.get_logger().info('5초 동안 물체가 인식되지 않았습니다. 노드를 종료합니다.')
#                 rclpy.shutdown()

#         cv2.imshow('Hand Gesture', img)
#         cv2.waitKey(100)

#     def detect_objects(self, img):
#         h, w = img.shape[:2]
#         newCameraMatrixL, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
#         frame_undistorted = cv2.undistort(img, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
#         brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.65, beta=0)
#         predictions = self.model.predict(brightness_adjusted[0:330, 68:570], conf=0.62)
#         results = predictions
#         boxes = results[0].boxes
#         all_boxes = boxes.xyxy.cpu().detach().numpy().tolist()
#         object_coords = []

#         for box in all_boxes:
#             class_id = boxes.cls.cpu().detach().numpy().tolist()[0]
#             if class_id == 0:
#                 X1, Y1, X2, Y2 = box
#                 h = Y2 - Y1
#                 w = X2 - X1
#                 box_area = h * w
#                 if box_area > 1150:
#                     recdis = (X2 - X1) / 2
#                     rx1 = X1 + recdis
#                     ry1 = Y2 - recdis
#                     pixelcoord = [rx1, ry1]
#                     robotcoord = [265, 182]
#                     resultx = (robotcoord[0] - pixelcoord[0]) * 10 * 0.158
#                     resulty = (pixelcoord[1] - robotcoord[1]) * 10 * 0.158
#                     object_coords.append((float(resultx), float(resulty)))
                    
#         return object_coords

#     def send_goal(self, x, y):
#         goal_msg = HandGestureAction.Goal()
#         goal_msg.coordx = x
#         goal_msg.coordy = y

#         self._action_client.wait_for_server()

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('액션 목표가 거부되었습니다.')
#             return

#         self.get_logger().info('액션 목표가 수락되었습니다.')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'결과 수신: 성공 = {result.success}')

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f'피드백: 진행 상황 = {feedback_msg.progress}')

#     def run(self):
#         rclpy.spin(self)

#     def stop(self):
#         self.stop_thread = True
#         if self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = HandGestureActionClient()

#     client_thread = threading.Thread(target=node.run)
#     client_thread.start()

#     try:
#         while rclpy.ok():
#             pass
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.stop()
#         client_thread.join()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()




# ============================================================================================== #


# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from hand_gesture_msgs.action import HandGestureAction
# import cv2
# from cv_bridge import CvBridge
# import pickle
# from ultralytics import YOLO
# import threading
# import time

# class HandGestureActionClient(Node):
#     def __init__(self):
#         super().__init__('hand_gesture_action_client')
#         self._action_client = ActionClient(self, HandGestureAction, 'hand_gesture')
#         self.get_logger().error('서버 기다리는중........!!!!!!!!!!!')
#         # 카메라 초기화 및 프레임 캡처 스레드
#         self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#         if not self.cap.isOpened():
#             self.get_logger().error('카메라를 열 수 없습니다.')
#             return
        
#         self.frame = None
#         self.frame_lock = threading.Lock()
#         self.frame_counter = 0
#         self.sample_interval = 5  # 매 5번째 프레임마다 처리
#         self.stop_thread = False
#         self.camera_thread = threading.Thread(target=self.capture_frame)
#         self.camera_thread.start()
        
#         self.bridge = CvBridge()

#         self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
#         self.model.verbose = False
#         with open("/home/lee/Desktop/topic_test/calibration_data.pkl", "rb") as f:
#             calibration_data = pickle.load(f)
#             self.cameraMatrixL = calibration_data["cameraMatrixL"]
#             self.distL = calibration_data["distL"]

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.last_sent_time = time.time()
#         self.cooldown_period = 20  # 20초 대기 시간 설정
#         self.object_coords = []
#         self.last_detection_time = time.time()  # 마지막 물체 인식 시간 초기화

#         # 액션 서버가 준비될 때까지 대기
#         self._action_client.wait_for_server()
        
#     def capture_frame(self):
#         while not self.stop_thread:
#             ret, img = self.cap.read()
#             if not ret:
#                 self.get_logger().error('카메라에서 프레임을 캡처하지 못했습니다.')
#                 continue
#             with self.frame_lock:
#                 self.frame = img
#             time.sleep(0.1)  # Adjust sleep time if needed

#     def timer_callback(self):
#         current_time = time.time()

#         if self.frame is None:
#             return

#         with self.frame_lock:
#             self.frame_counter += 1
#             if self.frame_counter % self.sample_interval != 0:
#                 return  # Skip this frame based on the sampling interval
            
#             img = self.frame.copy()  # Process the current frame
        
#         # 인식된 물체 좌표 얻기
#         coordinates = self.detect_objects(img)
#         self.object_coords = coordinates

#         if coordinates:
#             self.last_detection_time = current_time  # 마지막 인식 시간 갱신
#             if current_time - self.last_sent_time >= self.cooldown_period:
#                 # 각 물체 좌표 전송
#                 x, y = self.object_coords[0]
#                 self.send_goal(x, y)
#                 self.last_sent_time = current_time  # 마지막 전송 시간 갱신
#         else:
#             # 물체 인식되지 않은 시간 계산
#             if current_time - self.last_detection_time >= 60:
#                 self.get_logger().info('1분 동안 물체가 인식되지 않았습니다. 노드를 종료합니다.')
#                 rclpy.shutdown()

#         cv2.imshow('Hand Gesture', img)
#         cv2.waitKey(100)

#     def detect_objects(self, img):
#         h, w = img.shape[:2]
#         newCameraMatrixL, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
#         frame_undistorted = cv2.undistort(img, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
#         brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.65, beta=0)
#         predictions = self.model.predict(brightness_adjusted[0:330, 68:570], conf=0.62)
#         results = predictions
#         boxes = results[0].boxes
#         all_boxes = boxes.xyxy.cpu().detach().numpy().tolist()
#         object_coords = []

#         for box in all_boxes:
#             class_id = boxes.cls.cpu().detach().numpy().tolist()[0]
#             if class_id == 0:
#                 X1, Y1, X2, Y2 = box
#                 h = Y2 - Y1
#                 w = X2 - X1
#                 box_area = h * w
#                 if box_area > 1150:
#                     recdis = (X2 - X1) / 2
#                     rx1 = X1 + recdis
#                     ry1 = Y2 - recdis
#                     pixelcoord = [rx1, ry1]
#                     robotcoord = [265, 182]
#                     resultx = (robotcoord[0] - pixelcoord[0]) * 10 * 0.158
#                     resulty = (pixelcoord[1] - robotcoord[1]) * 10 * 0.158
#                     object_coords.append((float(resultx), float(resulty)))
                    
#         return object_coords

#     def send_goal(self, x, y):
#         goal_msg = HandGestureAction.Goal()
#         goal_msg.coordx = x
#         goal_msg.coordy = y

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('액션 목표가 거부되었습니다.')
#             return

#         self.get_logger().info('액션 목표가 수락되었습니다.')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'결과 수신: 성공 = {result.success}')

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f'피드백: 진행 상황 = {feedback_msg.progress}')

#     def run(self):
#         rclpy.spin(self)

#     def stop(self):
#         self.stop_thread = True
#         if self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = HandGestureActionClient()

#     client_thread = threading.Thread(target=node.run)
#     client_thread.start()

#     try:
#         while rclpy.ok():
#             pass
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.stop()
#         client_thread.join()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# ============================================================================================== #
# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from hand_gesture_msgs.action import HandGestureAction
# import cv2
# from cv_bridge import CvBridge
# import pickle
# from ultralytics import YOLO
# import torch
# import threading
# import time

# class HandGestureActionClient(Node):
#     def __init__(self):
#         super().__init__('hand_gesture_action_client')
#         self._action_client = ActionClient(self, HandGestureAction, 'hand_gesture')
        
#         # 카메라 초기화 및 프레임 캡처 스레드
#         self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#         if not self.cap.isOpened():
#             self.get_logger().error('카메라를 열 수 없습니다.')
#             return
        
#         self.frame = None
#         self.frame_lock = threading.Lock()
#         self.frame_counter = 0
#         self.sample_interval = 5  # 매 5번째 프레임마다 처리
#         self.stop_thread = False
#         self.camera_thread = threading.Thread(target=self.capture_frame)
#         self.camera_thread.start()
        
#         self.bridge = CvBridge()

#         self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
#         # self.model.verbose = False  # 메시지 억제
#         if torch.cuda.is_available():
#             self.model.to('cuda')  # GPU 사용
        
#         with open("/home/lee/Desktop/topic_test/calibration_data.pkl", "rb") as f:
#             calibration_data = pickle.load(f)
#             self.cameraMatrixL = calibration_data["cameraMatrixL"]
#             self.distL = calibration_data["distL"]

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.last_sent_time = time.time()
#         self.cooldown_period = 20  # 20초 대기 시간 설정
#         self.object_coords = []
#         self.last_detection_time = time.time()  # 마지막 물체 인식 시간 초기화

#         # # 액션 서버가 준비될 때까지 대기
#         # while not self._action_client.wait_for_server(timeout_sec=1.0):
#         #     self.get_logger().info('액션 서버를 기다리는 중...')

#     def capture_frame(self):
#         while not self.stop_thread:
#             ret, img = self.cap.read()
#             if not ret:
#                 self.get_logger().error('카메라에서 프레임을 캡처하지 못했습니다.')
#                 continue
#             with self.frame_lock:
#                 self.frame = img
#             time.sleep(0.1)  # Adjust sleep time if needed

#     def timer_callback(self):
#         current_time = time.time()

#         if self.frame is None:
#             return

#         with self.frame_lock:
#             self.frame_counter += 1
#             if self.frame_counter % self.sample_interval != 0:
#                 return  # Skip this frame based on the sampling interval
            
#             img = self.frame.copy()  # Process the current frame
        
#         # 인식된 물체 좌표 얻기
#         coordinates = self.detect_objects(img)
#         self.object_coords = coordinates

#         if coordinates:
#             self.last_detection_time = current_time  # 마지막 인식 시간 갱신
#             if current_time - self.last_sent_time >= self.cooldown_period:
#                 # 각 물체 좌표 전송
#                 x, y = self.object_coords[0]
#                 self.send_goal(x, y)
#                 self.last_sent_time = current_time  # 마지막 전송 시간 갱신
#         else:
#             # 물체 인식되지 않은 시간 계산
#             if current_time - self.last_detection_time >= 600:
#                 self.get_logger().info('600초 동안 물체가 인식되지 않았습니다. 노드를 종료합니다.')
#                 rclpy.shutdown()

#         cv2.imshow('Hand Gesture', img)
#         cv2.waitKey(100)

#     def detect_objects(self, img):
#         h, w = img.shape[:2]
#         newCameraMatrixL, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
#         frame_undistorted = cv2.undistort(img, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
#         brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.65, beta=0)
        
#         # 이미지 크기 줄이기
#         small_img = cv2.resize(brightness_adjusted, (320, 240))
        
#         predictions = self.model.predict(small_img, conf=0.62)
#         results = predictions
#         boxes = results[0].boxes
#         all_boxes = boxes.xyxy.cpu().detach().numpy().tolist()
#         object_coords = []

#         for box in all_boxes:
#             class_id = boxes.cls.cpu().detach().numpy().tolist()[0]
#             if class_id == 0:
#                 X1, Y1, X2, Y2 = box
#                 h = Y2 - Y1
#                 w = X2 - X1
#                 box_area = h * w
#                 if box_area > 1150:
#                     recdis = (X2 - X1) / 2
#                     rx1 = X1 + recdis
#                     ry1 = Y2 - recdis
#                     pixelcoord = [rx1, ry1]
#                     robotcoord = [265, 182]
#                     resultx = (robotcoord[0] - pixelcoord[0]) * 10 * 0.158
#                     resulty = (pixelcoord[1] - robotcoord[1]) * 10 * 0.158
#                     object_coords.append((float(resultx), float(resulty)))
#                     self.get_logger().info(f"object_coords : {object_coords}")
#         return object_coords

#     def send_goal(self, x, y):
#         goal_msg = HandGestureAction.Goal()
#         goal_msg.coordx = x
#         goal_msg.coordy = y

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('액션 목표가 거부되었습니다.')
#             return

#         self.get_logger().info('액션 목표가 수락되었습니다.')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'결과 수신: 성공 = {result.success}')

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f'피드백: 진행 상황 = {feedback_msg.progress}')

#     def run(self):
#         rclpy.spin(self)

#     def stop(self):
#         self.stop_thread = True
#         if self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = HandGestureActionClient()

#     client_thread = threading.Thread(target=node.run)
#     client_thread.start()

#     try:
#         while rclpy.ok():
#             pass
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.stop()
#         client_thread.join()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()



# ==========================================================================================

# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from hand_gesture_msgs.action import HandGestureAction
# import cv2
# from cv_bridge import CvBridge
# import pickle
# from ultralytics import YOLO
# import torch
# import threading
# import time
# import numpy as np

# class KalmanFilter:
#     def __init__(self):
#         # KalmanFilter 클래스 초기화
#         self.kf = cv2.KalmanFilter(4, 2)
        
#         # 상태 벡터 [x, y, dx, dy]
#         self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
#                                             [0, 1, 0, 1],
#                                             [0, 0, 1, 0],
#                                             [0, 0, 0, 1]], np.float32)
        
#         # 측정 행렬
#         self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
#                                               [0, 1, 0, 0]], np.float32)
        
#         # 프로세스 잡음 공분산 행렬
#         self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        
#         # 측정 잡음 공분산 행렬
#         self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        
#         # 초기 상태
#         self.kf.statePost = np.zeros((4, 1), np.float32)

#     def predict(self):
#         self.kf.predict()
#         return self.kf.statePost

#     def correct(self, measurement):
#         self.kf.correct(np.array(measurement, np.float32).reshape(2, 1))
#         return self.kf.statePost

# class HandGestureActionClient(Node):
#     def __init__(self):
#         super().__init__('hand_gesture_action_client')
#         self._action_client = ActionClient(self, HandGestureAction, 'hand_gesture')

#         # 카메라 초기화 및 프레임 캡처 스레드
#         self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#         if not self.cap.isOpened():
#             self.get_logger().error('카메라를 열 수 없습니다.')
#             return

#         self.frame = None
#         self.frame_lock = threading.Lock()
#         self.frame_counter = 0
#         self.sample_interval = 5  # 매 5번째 프레임마다 처리
#         self.stop_thread = False
#         self.camera_thread = threading.Thread(target=self.capture_frame)
#         self.camera_thread.start()

#         self.bridge = CvBridge()

#         self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
        
#         if torch.cuda.is_available():
#             self.model.to('cuda')  # GPU 사용

#         with open("/home/lee/Desktop/topic_test/calibration_data.pkl", "rb") as f:
#             calibration_data = pickle.load(f)
#             self.cameraMatrixL = calibration_data["cameraMatrixL"]
#             self.distL = calibration_data["distL"]

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.last_sent_time = time.time()
#         self.cooldown_period = 20  # 20초 대기 시간 설정
#         self.object_coords = []
#         self.last_detection_time = time.time()  # 마지막 물체 인식 시간 초기화

#         # Kalman 필터 인스턴스 생성
#         self.kalman_filter = KalmanFilter()

#     def capture_frame(self):
#         while not self.stop_thread:
#             ret, img = self.cap.read()
#             if not ret:
#                 self.get_logger().error('카메라에서 프레임을 캡처하지 못했습니다.')
#                 continue
#             with self.frame_lock:
#                 self.frame = img
#             time.sleep(0.1)  # Adjust sleep time if needed

#     def timer_callback(self):
#         current_time = time.time()

#         if self.frame is None:
#             return

#         with self.frame_lock:
#             self.frame_counter += 1
#             if self.frame_counter % self.sample_interval != 0:
#                 return  # Skip this frame based on the sampling interval

#             img = self.frame.copy()  # Process the current frame

#         # 인식된 물체 좌표 얻기
#         coordinates = self.detect_objects(img)
#         if coordinates:
#             x, y = coordinates[0]
#             # Kalman 필터에 좌표 전달
#             predicted_state = self.kalman_filter.predict()
#             corrected_state = self.kalman_filter.correct([x, y])
#             x, y = corrected_state[0, 0], corrected_state[1, 0]

#             self.last_detection_time = current_time  # 마지막 인식 시간 갱신
#             if current_time - self.last_sent_time >= self.cooldown_period:
#                 # 각 물체 좌표 전송
#                 self.send_goal(x, y)
#                 self.last_sent_time = current_time  # 마지막 전송 시간 갱신
#         else:
#             # 물체 인식되지 않은 시간 계산
#             if current_time - self.last_detection_time >= 600:
#                 self.get_logger().info('600초 동안 물체가 인식되지 않았습니다. 노드를 종료합니다.')
#                 rclpy.shutdown()

#         cv2.imshow('Hand Gesture', img)
#         cv2.waitKey(100)

#     def detect_objects(self, img):
#         h, w = img.shape[:2]
#         newCameraMatrixL, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
#         frame_undistorted = cv2.undistort(img, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
#         brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.65, beta=0)

#         # 이미지 크기 줄이기
#         small_img = cv2.resize(brightness_adjusted, (320, 240))

#         predictions = self.model.predict(small_img, conf=0.62)
#         results = predictions
#         boxes = results[0].boxes
#         all_boxes = boxes.xyxy.cpu().detach().numpy().tolist()
#         object_coords = []

#         for box in all_boxes:
#             class_id = boxes.cls.cpu().detach().numpy().tolist()[0]
#             if class_id == 0:
#                 X1, Y1, X2, Y2 = box
#                 h = Y2 - Y1
#                 w = X2 - X1
#                 box_area = h * w
#                 if box_area > 1150:
#                     recdis = (X2 - X1) / 2
#                     rx1 = X1 + recdis
#                     ry1 = Y2 - recdis
#                     pixelcoord = [rx1, ry1]
#                     robotcoord = [265, 182]
#                     resultx = (robotcoord[0] - pixelcoord[0]) * 10 * 0.158
#                     resulty = (pixelcoord[1] - robotcoord[1]) * 10 * 0.158
#                     object_coords.append((float(resultx), float(resulty)))
#                     self.get_logger().info(f"object_coords : {object_coords}")
#         return object_coords

#     def send_goal(self, x, y):
#         goal_msg = HandGestureAction.Goal()
#         goal_msg.coordx = x
#         goal_msg.coordy = y

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('액션 목표가 거부되었습니다.')
#             return

#         self.get_logger().info('액션 목표가 수락되었습니다.')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'결과 수신: 성공 = {result.success}')

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f'피드백: 진행 상황 = {feedback_msg.progress}')

#     def run(self):
#         rclpy.spin(self)

#     def stop(self):
#         self.stop_thread = True
#         if self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = HandGestureActionClient()

#     client_thread = threading.Thread(target=node.run)
#     client_thread.start()

#     try:
#         while rclpy.ok():
#             pass
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.stop()
#         client_thread.join()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# ========================================================================================================================


# import rclpy
# from rclpy.action import ActionClient
# from rclpy.node import Node
# from hand_gesture_msgs.action import HandGestureAction
# import cv2
# from cv_bridge import CvBridge
# import pickle
# from ultralytics import YOLO
# import torch
# import threading
# import time
# import numpy as np

# class KalmanFilter:
#     def __init__(self):
#         # KalmanFilter 클래스 초기화
#         self.kf = cv2.KalmanFilter(4, 2)
        
#         # 상태 벡터 [x, y, dx, dy]
#         self.kf.transitionMatrix = np.array([[1, 0, 1, 0],
#                                             [0, 1, 0, 1],
#                                             [0, 0, 1, 0],
#                                             [0, 0, 0, 1]], np.float32)
        
#         # 측정 행렬
#         self.kf.measurementMatrix = np.array([[1, 0, 0, 0],
#                                               [0, 1, 0, 0]], np.float32)
        
#         # 프로세스 잡음 공분산 행렬
#         self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        
#         # 측정 잡음 공분산 행렬
#         self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        
#         # 초기 상태
#         self.kf.statePost = np.zeros((4, 1), np.float32)

#     def predict(self):
#         self.kf.predict()
#         return self.kf.statePost

#     def correct(self, measurement):
#         self.kf.correct(np.array(measurement, np.float32).reshape(2, 1))
#         return self.kf.statePost

# class HandGestureActionClient(Node):
#     def __init__(self):
#         super().__init__('hand_gesture_action_client')
#         self._action_client = ActionClient(self, HandGestureAction, 'hand_gesture')

#         # 카메라 초기화 및 프레임 캡처 스레드
#         self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#         if not self.cap.isOpened():
#             self.get_logger().error('카메라를 열 수 없습니다.')
#             return

#         self.frame = None
#         self.frame_lock = threading.Lock()
#         self.frame_counter = 0
#         self.sample_interval = 5  # 매 5번째 프레임마다 처리
#         self.stop_thread = False
#         self.camera_thread = threading.Thread(target=self.capture_frame)
#         self.camera_thread.start()

#         self.bridge = CvBridge()

#         self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
        
#         if torch.cuda.is_available():
#             self.model.to('cuda')  # GPU 사용

#         with open("/home/lee/Desktop/topic_test/calibration_data.pkl", "rb") as f:
#             calibration_data = pickle.load(f)
#             self.cameraMatrixL = calibration_data["cameraMatrixL"]
#             self.distL = calibration_data["distL"]

#         self.timer = self.create_timer(0.1, self.timer_callback)

#         self.last_sent_time = time.time()
#         self.cooldown_period = 20  # 20초 대기 시간 설정
#         self.object_coords = []
#         self.last_detection_time = time.time()  # 마지막 물체 인식 시간 초기화

#         # Kalman 필터 인스턴스 생성
#         self.kalman_filter = KalmanFilter()

#     def capture_frame(self):
#         while not self.stop_thread:
#             ret, img = self.cap.read()
#             if not ret:
#                 self.get_logger().error('카메라에서 프레임을 캡처하지 못했습니다.')
#                 continue
#             with self.frame_lock:
#                 self.frame = img
#             time.sleep(0.1)  # Adjust sleep time if needed

#     def timer_callback(self):
#         current_time = time.time()

#         if self.frame is None:
#             return

#         with self.frame_lock:
#             self.frame_counter += 1
#             if self.frame_counter % self.sample_interval != 0:
#                 return  # Skip this frame based on the sampling interval

#             img = self.frame.copy()  # Process the current frame

#         # 인식된 물체 좌표 얻기
#         coordinates = self.detect_objects(img)
#         if coordinates:
#             x, y = coordinates[0]
#             # Kalman 필터에 좌표 전달
#             predicted_state = self.kalman_filter.predict()
#             corrected_state = self.kalman_filter.correct([x, y])
#             x, y = corrected_state[0, 0], corrected_state[1, 0]

#             self.last_detection_time = current_time  # 마지막 인식 시간 갱신
#             if current_time - self.last_sent_time >= self.cooldown_period:
#                 # 각 물체 좌표 전송
#                 self.send_goal(x, y)
#                 self.last_sent_time = current_time  # 마지막 전송 시간 갱신
#         else:
#             # 물체 인식되지 않은 시간 계산
#             if current_time - self.last_detection_time >= 600:
#                 self.get_logger().info('600초 동안 물체가 인식되지 않았습니다. 노드를 종료합니다.')
#                 rclpy.shutdown()

#     def detect_objects(self, img):
#         h, w = img.shape[:2]
#         newCameraMatrixL, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
#         frame_undistorted = cv2.undistort(img, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
#         brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.65, beta=0)

#         # 이미지 크기 줄이기
#         small_img = cv2.resize(brightness_adjusted, (320, 240))

#         predictions = self.model.predict(small_img, conf=0.62)
#         results = predictions
#         boxes = results[0].boxes
#         all_boxes = boxes.xyxy.cpu().detach().numpy().tolist()
#         object_coords = []

#         for box in all_boxes:
#             class_id = boxes.cls.cpu().detach().numpy().tolist()[0]
#             if class_id == 0:
#                 X1, Y1, X2, Y2 = box
#                 h = Y2 - Y1
#                 w = X2 - X1
#                 box_area = h * w
#                 if box_area > 1150:
#                     recdis = (X2 - X1) / 2
#                     rx1 = X1 + recdis
#                     ry1 = Y2 - recdis
#                     pixelcoord = [rx1, ry1]
#                     robotcoord = [265, 182]
#                     resultx = (robotcoord[0] - pixelcoord[0]) * 10 * 0.158
#                     resulty = (pixelcoord[1] - robotcoord[1]) * 10 * 0.158
#                     object_coords.append((float(resultx), float(resulty)))
#                     self.get_logger().info(f"object_coords : {object_coords}")
#         return object_coords

#     def send_goal(self, x, y):
#         goal_msg = HandGestureAction.Goal()
#         goal_msg.coordx = x
#         goal_msg.coordy = y

#         self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
#         self._send_goal_future.add_done_callback(self.goal_response_callback)

#     def goal_response_callback(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().info('액션 목표가 거부되었습니다.')
#             return

#         self.get_logger().info('액션 목표가 수락되었습니다.')
#         self._get_result_future = goal_handle.get_result_async()
#         self._get_result_future.add_done_callback(self.get_result_callback)

#     def get_result_callback(self, future):
#         result = future.result().result
#         self.get_logger().info(f'결과 수신: 성공 = {result.success}')

#     def feedback_callback(self, feedback_msg):
#         self.get_logger().info(f'피드백: 진행 상황 = {feedback_msg.progress}')

#     def run(self):
#         rclpy.spin(self)

#     def stop(self):
#         self.stop_thread = True
#         if self.cap is not None:
#             self.cap.release()
#         cv2.destroyAllWindows()

# def main(args=None):
#     rclpy.init(args=args)
#     node = HandGestureActionClient()

#     client_thread = threading.Thread(target=node.run)
#     client_thread.start()

#     try:
#         while rclpy.ok():
#             pass
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.stop()
#         client_thread.join()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# =========================================================================================================

# import rclpy as rp
# from rclpy.node import Node
# from hand_gesture_msgs.srv import Check
# import time
# from hand_gesture_pkg.scripts_ver2 import RobotMain
# from xarm.wrapper import XArmAPI
# from std_msgs.msg import Bool

# class HandGestureActionClient(Node):
    
#     def __init__(self):
#         super().__init__('hand_gesture_action_client')
    
#         self.client = self.create_client(Check, 'get_coord') # 같은 서비스 이름 타입
     
#         while not self.client.wait_for_service(timeout_sec=1.0):
#             self.get_logger().info('Service not available, waiting again...')
        
#         self.request = Check.Request()
#         self.timer_period = 40 # 요청을 보낼 주기 (초 단위)
#         self.timer = self.create_timer(self.timer_period, self.send_request)
#         self.future = None
#         self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
#         self.robot_main = RobotMain(self.arm)
     
#                 # 상태 구독자 설정
#         self.robot_status_subscriber = self.create_subscription(
#             Bool,
#             'robot_to_kiosk_status',
#             self.robot_status_callback,
#             10
#         )
#         self.robot_active = True
     
#     def robot_status_callback(self, msg):
#         """로봇 상태를 구독하여 활성화 상태를 업데이트합니다."""
#         self.robot_active = msg.data
     
     
#     # 요청 보내기 함수 
#     def send_request(self):    
#         self.request.signal = 1
#         self.future = self.client.call_async(self.request)
#         self.get_logger().info('Request sent')
    
#     def check_response(self):
#         if self.future.done():
#             if self.robot_active:
#                 self.get_logger().info('RobotToKiosk 노드가 활성 상태입니다. HandGestureActionServer가 작동하지 않습니다.')
            
#             else:
#                 try:
#                     response = self.future.result()
#                     self.get_logger().info(f'Result: x={response.x}, y={response.y}')
#                     self.robot_main.trash(response.x, response.y)
#                 except Exception as e:
#                     self.get_logger().info(f'Service call failed: {e}')
#                 else:
#                     self.get_logger().info('else: None!!!!!!!!!!!!!!!!!!!!!111')
                
#             # 다시 요청을 보내기 위해 future 초기화
#             self.future = None

# def main(args=None):
#     rp.init(args=args)
#     coordinate_client = HandGestureActionClient() # 고객 노드 생성 
    
#     while rp.ok():
#         rp.spin_once(coordinate_client, timeout_sec=1.0)
#         if coordinate_client.future:
#             coordinate_client.check_response()
#             time.sleep(5) 
            
#     coordinate_client.destroy_node()
#     rp.shutdown()

# if __name__ == '__main__':
#     main()
      
      
      
      
import rclpy as rp
from rclpy.node import Node
from hand_gesture_msgs.srv import Check
from std_msgs.msg import Bool
from xarm.wrapper import XArmAPI
from hand_gesture_pkg.scripts_ver2 import RobotMain
import time

class HandGestureActionClient(Node):
    
    def __init__(self):
        super().__init__('hand_gesture_action_client')
    
        self.client = self.create_client(Check, 'get_coord')  # 서비스 클라이언트 생성
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        
        self.request = Check.Request()
        self.timer_period = 40  # 요청을 보낼 주기 (초 단위)
        self.timer = self.create_timer(self.timer_period, self.send_request)
        self.future = None
        self.arm = XArmAPI('192.168.1.184', baud_checkset=False)
        self.robot_main = RobotMain(self.arm)
     
        # 상태 구독자 설정
        self.robot_status_subscriber = self.create_subscription(
            Bool,
            'robot_to_kiosk_status',
            self.robot_status_callback,
            10
        )
        self.robot_active = False  # 초기 상태를 False로 설정
     
    def robot_status_callback(self, msg):
        """로봇 상태를 구독하여 활성화 상태를 업데이트합니다."""
        self.robot_active = msg.data
     
    def send_request(self):    
        if not self.robot_active:  # 로봇이 비활성 상태일 때만 요청을 보냅니다.
            self.request.signal = 1
            self.future = self.client.call_async(self.request)
            self.get_logger().info('Request sent')
    
    def check_response(self):
        if self.future and self.future.done():
            if self.robot_active:
                self.get_logger().info(f'self.robot_active : {self.robot_active}')
                self.get_logger().info('RobotToKiosk 노드가 활성 상태입니다. 요청을 처리하지 않습니다.')
            else:
                try:
                    self.get_logger().info(f'self.robot_active : {self.robot_active}')
                    response = self.future.result()
                    self.get_logger().info(f'Result: x={response.x}, y={response.y}')
                    self.robot_main.trash(response.x, response.y)
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                
            # 다시 요청을 보내기 위해 future 초기화
            self.future = None

def main(args=None):
    rp.init(args=args)
    coordinate_client = HandGestureActionClient()  # 클라이언트 노드 생성 
    
    while rp.ok():
        rp.spin_once(coordinate_client, timeout_sec=1.0)
        coordinate_client.check_response()
        time.sleep(5)  # 적절한 주기로 대기
        
    coordinate_client.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
