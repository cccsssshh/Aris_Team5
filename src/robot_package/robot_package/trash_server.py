#############<cam>############
import cv2
import numpy as np
from ultralytics import YOLO
import time
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
##############<ros>############
import rclpy as rp
from rclpy.node import Node
from interface_package.srv import Check
from std_msgs.msg import Bool

##############<calibration>#####################
import glob
import pickle
class trash_server(Node):

    # 시작하자마자 실행
    def __init__(self):
        super().__init__('trash_server')
        ########################기본################################
        self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
        self.bounding_boxes = []
        self.capL = cv2.VideoCapture(0)
        self.capL.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capL.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        ### 초기화 ###
        self.count = 0
        self.removepoint = 0
        self.cup_coordinates = [] #평균치 데이터 만들기
        self.robot_coords = [265, 182]
        self.cameraMatrixL = None
        self.dist = None 
        self.avecoord = []
        self.savecoord = [] ### 보낼 데이터 저장 ###
        self.clustermul=[]
        self.clusterdiv=[]
        self.clustersave=[] ####cluster변경 #####
        
        
        self.srv = self.create_service(Check, 'get_coord', self.service_callback)
        
        #################### 체크보드 값 기본값 ###############################
        with open("calibration_data.pkl", "rb") as f:
            calibration_data = pickle.load(f)

        self.cameraMatrixL = calibration_data["cameraMatrixL"]
        self.distL = calibration_data["distL"]

        ###########################################################################


    # 카메라 보정
    # 카메라 키고 보정모델 돌려서 위치 저장고 만듬  
    def capture_frame(self):
       #서비스가 연속으로 부를때를 대비하여 부르기
       self.count = 0
       self.avecoord = []
       self.clusterready=[]
       self.clusterdiv = []
       self.clustersave = []
       self.savecoord = []

       while True: 
        #while 반복초기화
            self.cups_detected = 0
            self.cup_coordinates = []
            self.removepoint = 0
        #캠키기 준비    
            ret, frameL = self.capL.read()
            if not ret:
                self.get_logger().error("Failed to capture frame.")
                return
        # 왜곡보정 
            h, w = frameL.shape[:2]
            newCameraMatrixL,roiL = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
            frame_undistorted = cv2.undistort(frameL, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
        # 밝기 조절
            brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.68, beta=0)
            predictions = self.model(brightness_adjusted[18:300, 68:570])
            results = predictions
        # 모델 돌린 결과 값 
            allbox = results[0].boxes.xyxy.cpu().detach().numpy().tolist() 
            #값이 존재?
            if not allbox:
                return                
            #컵 개수를 알 수 있는 값
            for result in results:
                boxes = result.boxes
                self.cups_detected += len(boxes)
            
            # 각 다른 컵 좌표 등록      
            cup = boxes.xyxy.cpu().detach().numpy().tolist()
            for i in range(self.cups_detected):
                X1, Y1, X2, Y2 = cup[i][0], cup[i][1], cup[i][2], cup[i][3]
                #좌표 변환 + 박스 제한 
                h = Y2 - Y1
                w = X2 - X1
                box_XX = h*w
                if box_XX > 1190 :
                    recdis = (X2 - X1)/2 
                    rx1 = X1 + recdis
                    ry1 = Y2 - recdis
                    pixelcoord = [rx1, ry1]
                    robotcoord = [265, 182]
                    resultx = (robotcoord[0] - pixelcoord[0])*10 * 0.158                    
                    resulty = (pixelcoord[1] - robotcoord[1])*10 * 0.158 
                    result = [resultx,resulty]
                    self.cup_coordinates.append(result)                            
                else :
                    self.removepoint += 1
                #[[12 34] [32 45] [67 34]] 한 개 생성        
           
            #컵이 존재하나 다 가려진 컵들이야 
            if not self.cup_coordinates:
               break             

            #좌표 저장고에 붙이기 --> 평균 만들기 위함 
            self.avecoord.append(self.cup_coordinates)
            self.count = self.count +1 
            
            if self.count == 3 : 
                break 
        
        #[[[12 34] [32 45] [67 34]] 여러 개 생성 while 한번 당 한줄
        #[[12 34] [32 45] [67 34]]
        #[[12 34] [32 45] [67 34]]]
    ############################################################################################333
    #<정리>클러스트링 이용    
      #예제 데이터
       self.clustersave = []
       self.savecoord = [item for sublist in self.avecoord for item in sublist]    # 이중리스트로 만들기   
       self.clustermul = [[x * 5 for x in sublist] for sublist in self.savecoord]  # 구분을 쉽게 하기 위함
       print("저장소",self.clustermul)
      #K-Means 클러스터링을 사용하여 데이터 군집화
       self.cups_detected = self.cups_detected - self.removepoint
       print(self.cups_detected)
      #가려진 컵이 일부가 있음   
       if self.cups_detected == 0:
          return 
       kmeans = KMeans(n_clusters=self.cups_detected) # 몇가지로 분류
       kmeans.fit(self.clustermul)
       labels = kmeans.labels_
       # 클러스터 별 데이터 묶기
       clusters = {i: [] for i in range(self.cups_detected)}
       for idx, label in enumerate(labels):
           clusters[label].append(self.clustermul[idx])
        
       print(self.clustermul)
       #비슷한 거 끼리 모음 
       #평균 만들기 -->좌표 저장 
       
       for i in range (self.cups_detected):
          cluster_0_data = np.array(clusters[i])
          cluster_0_mean = np.mean(cluster_0_data, axis=0)
          self.clusterdiv.append(cluster_0_mean) 
       
       self.clustersave = [array / 5 for array in self.clusterdiv]

       
       print("평균화",self.clustersave)
       return self.clustersave   
    
    # clustersave를 차례대로 보내기 위해서 필요한 애 
    def coordrequest(self):
        #요청 하나 받을 때 마다 저장된 함수
        if len(self.clustersave) == 0 :
           self.capture_frame()
        else:
            del self.clustersave[0]  
            print(self.clustersave)
            return self.clustersave
        

    # 실제 좌표 리턴 서비스 
    def service_callback(self, request, response):        
        self.coordrequest()
        #(case)
        #좌표 저장된게 없음
        if self.cups_detected == 0 :
            response.x, response.y = 0.0, 0.0
            self.get_logger().info('No valid coordinates found.')
            return response
        #컵이 있으나 다가려진 경우
        if not self.clustersave:
            response.x, response.y = 0.0 , 0.0
            self.get_logger().info('No valid coordinates found.')
            return response
        #신호 들어옴 
        if request.signal == 1:
            #값 넣어서 반응에 보내기   
            response.x = self.clustersave[0][0]
            response.y = self.clustersave[0][1] 
            if response.x and response.y :
                self.get_logger().info(f'Predicted coordinates: x={response.x}, y={response.y}')
                return response
            else :
                response.x, response.y = 0.0, 0.0
                self.get_logger().info('No valid coordinates found.')
                return response 
        else:  
            response.x, response.y = 0.0 , 0.0
            self.get_logger().info('No valid signal received.')
            return response

def main(args=None):
    rp.init(args=args)
    BinCupxyserver = trash_server()
    rp.spin(BinCupxyserver)

    BinCupxyserver.destroy_node()
    rp.shutdown()

if __name__ == '__main__':
    main()
