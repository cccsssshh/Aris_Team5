import sys
import os
from ament_index_python.packages import get_package_share_directory
# 현재 스크립트의 디렉토리를 sys.path에 추가
current_dir = os.path.dirname(os.path.abspath(__file__))
package_share_directory = get_package_share_directory('kiosk_package')
gesture_model_file_path = os.path.join(package_share_directory, 'models/gesture_model_v2.hdf5')
age_model_file_path = os.path.join(package_share_directory, 'models/age_model.hdf5')

import cv2
import numpy as np
from deepface import DeepFace
from tensorflow.keras.preprocessing import image
from tensorflow.keras.applications.vgg16 import preprocess_input
from tensorflow.keras.models import load_model
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
import mediapipe as mp
from collections import Counter
import time
from collections import deque

os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/local/opt/qt/plugins'  # 이 경로를 실제 Qt 플러그인 경로로 설정



class GestureModel():
    def __init__(self):
        self.model = load_model(gesture_model_file_path)  # 동작 인식 모델
        self.this_action = '?'  # 현재 동작
        self.actions = ['1', '2', '3', 'O', 'X']  # 가능한 동작 리스트
        self.seq_length = 30  # 시퀀스 길이
        self.seq = []  # 관절 데이터 시퀀스
        self.consistent_action_threshold = 15
        self.confidence_threshold = 0.95
        self.time_threshold = 0.5  # seconds
        self.action_seq = deque(maxlen=self.consistent_action_threshold)
        self.action_start_time = None
        self.action_probs = []  # 예측 확률 누적 리스트
        self.avg_action = None  # 평균 액션
        self.prev_landmarks = None  # 이전 프레임의 랜드마크
        self.mp_hands = mp.solutions.hands  # 미디어파이프 손 모듈
        self.mp_drawing = mp.solutions.drawing_utils  # 미디어파이프 그리기 도구
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.8
        )
    def process_landmarks(self, landmarks):
        return [[lm.x * 100, lm.y * 100, lm.z * 100] for lm in landmarks]
    
    def analyzeGesture(self, img):
        img = cv2.flip(img, 1)
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)  # BGR에서 RGB로 색상 공간 변환
        result = self.hands.process(img_rgb)  # 이미지를 손 추적 네트워크에 입력하여 결과 받기

        if result.multi_hand_landmarks is not None:  # 손의 랜드마크가 있으면
            for res in result.multi_hand_landmarks:  # 각 손에 대해
                joint = np.array([[lm.x, lm.y, lm.z, lm.visibility] for lm in res.landmark])  # 21개의 랜드마크를 저장할 배열 생성

                if self.prev_landmarks is not None:
                    movement = np.linalg.norm(joint[:, :3] - self.prev_landmarks[:, :3], axis=1).mean()
                else:
                    movement = 0

                self.prev_landmarks = joint.copy()

                # 움직임이 일정 수준 이상이면 continue
                if movement > 0.02:
                    cv2.putText(img, "Hand Moving", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    self.seq.clear()
                    self.action_seq.clear()
                    self.action_probs.clear()
                    self.this_action = '?'
                    continue

                # Compute angles between joints
                v1 = joint[[0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 0, 17, 18, 19], :3]  # Parent joint
                v2 = joint[[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20], :3]  # Child joint
                v = v2 - v1  # [20, 3]
                # Normalize v
                v = v / np.linalg.norm(v, axis=1)[:, np.newaxis]  # 벡터 정규화

                # Get angle using arcos of dot product (내적의 arcos로 각도 구하기)
                angle = np.arccos(np.einsum('nt,nt->n',
                                            v[[0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18], :],
                                            v[[1, 2, 3, 5, 6, 7, 9, 10, 11, 13, 14, 15, 17, 18, 19], :]))  # [15,]

                angle = np.degrees(angle)  # Convert radian to degree

                d = np.concatenate([joint.flatten(), angle])  # 랜드마크와 각도 연결

                self.seq.append(d)  # 시퀀스에 추가

                self.mp_drawing.draw_landmarks(img, res, self.mp_hands.HAND_CONNECTIONS)  # 이미지에 랜드마크 그리기

                if len(self.seq) < self.seq_length:  # 시퀀스 길이가 충분하지 않으면 계속
                    continue

                input_data = np.expand_dims(np.array(self.seq[-self.seq_length:], dtype=np.float32), axis=0)

                # 모델 예측 시간 측정 시작
                start_time = time.time()
                y_pred = self.model.predict(input_data, verbose=0).squeeze()
                prediction_time = time.time() - start_time
                print(f"Prediction time: {prediction_time:.4f} seconds")  # 예측 시간 로그 출력

                i_pred = int(np.argmax(y_pred))
                conf = y_pred[i_pred]

                if conf < self.confidence_threshold:
                    self.action_start_time = None
                    self.action_seq.clear()  # Reset the action sequence as well
                    continue

                action = self.actions[i_pred]
                self.action_seq.append(action)

                if len(self.action_seq) == self.consistent_action_threshold:
                    if len(set(self.action_seq)) == 1:
                        current_time = time.time()
                        if self.action_start_time is None:
                            self.action_start_time = current_time
                        else:
                            if current_time - self.action_start_time >= self.time_threshold:
                                self.this_action = self.action_seq[0]
                                self.action_start_time = None  # Reset the timer after detecting the action
                    else:
                        self.action_start_time = None
                else:
                    self.action_start_time = None
            cv2.putText(img, f'{self.this_action.upper()}', org=(int(res.landmark[0].x * img.shape[1]), int(res.landmark[0].y * img.shape[0] + 20)), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1, color=(0, 0, 255), thickness=2)

        return img  # 처리된 이미지를 반환
    

class AgeModel():
    def __init__(self):
        self.model = load_model(age_model_file_path)
        self.ageLabels = ['00-09', '10-19', '20-29', '30-39', '40-49', '50-59', '60-100']
        self.frameCount = 0
        self.lastAgeLabel = ""
        self.lastGenderLabel = ""
        self.evaluated = False
        self.age = None
        self.gender = None
        self.ageBuffer = []
        self.genderBuffer = []

    def predictAge(self, faceImage):
        faceResized = cv2.resize(faceImage, (224, 224))
        faceArray = image.img_to_array(faceResized)
        faceArray = np.expand_dims(faceArray, axis=0)
        faceArray = preprocess_input(faceArray)
        
        preds = self.model.predict(faceArray)
        ageIndex = np.argmax(preds)
        ageLabel = self.ageLabels[ageIndex]
        ageConfidence = preds[0][ageIndex] * 100  # 나이대 확률
        
        return ageLabel, ageConfidence
        
    def analyzeAgeGender(self, frame):
        try:
            analysis = DeepFace.analyze(frame, actions=['gender'], detector_backend='yunet', enforce_detection=False)
            if isinstance(analysis, list):
                for face in analysis:
                    region = face['region']
                    x, y, w, h = region['x'], region['y'], region['w'], region['h']
                    face_image = frame[y:y+h, x:x+w]
                    
                    self.frameCount += 1
                    if self.frameCount % 10 == 0 or not self.evaluated:
                        # 사용자 정의 모델을 사용하여 나이 예측
                        ageLabel, ageConfidence = self.predictAge(face_image)
                        if ageConfidence >= 80:
                            self.lastAgeLabel = ageLabel
                            self.lastGenderLabel = f"Gender: {face['dominant_gender']}"
                            self.evaluated = True
                        else:
                            self.evaluated = False
                    
                    # 평가가 완료된 후에만 결과 표시
                    if self.evaluated:
                        ageLabel = f"Age: {self.lastAgeLabel}"
                        genderLabel = self.lastGenderLabel
                        self.ageBuffer.append(ageLabel)
                        self.genderBuffer.append(genderLabel)
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
                        cv2.putText(frame,ageLabel, (x, y-30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                        cv2.putText(frame, genderLabel, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
                    else:
                        cv2.rectangle(frame, (x, y), (x+w, y+h), (255, 0, 0), 2)
        except Exception as e:
            print(f"Error: {e}")
        
        return frame
    
    #결제 버튼 눌렀을 때 동작 
    def getMostCommonAgeGender(self):
        if not self.ageBuffer or not self.genderBuffer:
            return None, None
        
        ageCounter = Counter(self.ageBuffer)
        genderCounter = Counter(self.genderBuffer)
        
        mostCommonAge = ageCounter.most_common(1)[0][0]
        mostCommonGender = genderCounter.most_common(1)[0][0]
        
        self.ageBuffer = []
        self.genderBuffer = []

        return mostCommonAge, mostCommonGender

