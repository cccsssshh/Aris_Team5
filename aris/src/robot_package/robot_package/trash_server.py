import rclpy as rp
from rclpy.node import Node
from interface_package.srv import Check
import cv2
import numpy as np
from ultralytics import YOLO
from sklearn.cluster import KMeans
import pickle

class trash_server(Node):

    def __init__(self):
        super().__init__('trash_server')
        self.model = YOLO('/home/lee/Desktop/topic_test/best.pt')
        self.bounding_boxes = []
        self.capL = cv2.VideoCapture(1)
        self.capL.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capL.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.count = 0
        self.removepoint = 0
        self.cup_coordinates = []
        self.robot_coords = [265, 182]
        self.cameraMatrixL = None
        self.distL = None 
        self.avecoord = []
        self.savecoord = []
        self.clustermul=[]
        self.clusterdiv=[]
        self.clustersave=[]
        
        self.srv = self.create_service(Check, 'get_coord', self.service_callback)
        
        with open("calibration_data.pkl", "rb") as f:
            calibration_data = pickle.load(f)

        self.cameraMatrixL = calibration_data["cameraMatrixL"]
        self.distL = calibration_data["distL"]

    def capture_frame(self):
        self.count = 0
        self.avecoord = []
        self.clusterready=[]
        self.clusterdiv = []
        self.clustersave = []
        self.savecoord = []

        while True: 
            self.cups_detected = 0
            self.cup_coordinates = []
            self.removepoint = 0
            
            ret, frameL = self.capL.read()
            if not ret:
                self.get_logger().error("Failed to capture frame.")
                return
            
            h, w = frameL.shape[:2]
            newCameraMatrixL, roiL = cv2.getOptimalNewCameraMatrix(self.cameraMatrixL, self.distL, (w, h), 1, (w, h))
            frame_undistorted = cv2.undistort(frameL, self.cameraMatrixL, self.distL, None, newCameraMatrixL)
            brightness_adjusted = cv2.convertScaleAbs(frame_undistorted, alpha=0.68, beta=0)
            predictions = self.model(brightness_adjusted[18:300, 68:570])
            results = predictions
            
            allbox = results[0].boxes.xyxy.cpu().detach().numpy().tolist()
            if not allbox:
                return
                
            for result in results:
                boxes = result.boxes
                self.cups_detected += len(boxes)
            
            cup = boxes.xyxy.cpu().detach().numpy().tolist()
            for i in range(self.cups_detected):
                X1, Y1, X2, Y2 = cup[i][0], cup[i][1], cup[i][2], cup[i][3]
                h = Y2 - Y1
                w = X2 - X1
                box_XX = h * w
                if box_XX > 1190:
                    recdis = (X2 - X1) / 2 
                    rx1 = X1 + recdis
                    ry1 = Y2 - recdis
                    pixelcoord = [rx1, ry1]
                    robotcoord = [265, 182]
                    resultx = (robotcoord[0] - pixelcoord[0]) * 10 * 0.158                    
                    resulty = (pixelcoord[1] - robotcoord[1]) * 10 * 0.158 
                    result = [resultx, resulty]
                    self.cup_coordinates.append(result)
                else:
                    self.removepoint += 1

            if not self.cup_coordinates:
                break
                
            self.avecoord.append(self.cup_coordinates)
            self.count += 1 
            
            if self.count == 3:
                break 

        self.clustersave = []
        self.savecoord = [item for sublist in self.avecoord for item in sublist]
        self.clustermul = [[x * 5 for x in sublist] for sublist in self.savecoord]
        kmeans = KMeans(n_clusters=self.cups_detected)
        kmeans.fit(self.clustermul)
        labels = kmeans.labels_
        clusters = {i: [] for i in range(self.cups_detected)}
        for idx, label in enumerate(labels):
            clusters[label].append(self.clustermul[idx])
        
        for i in range(self.cups_detected):
            cluster_0_data = np.array(clusters[i])
            cluster_0_mean = np.mean(cluster_0_data, axis=0)
            self.clusterdiv.append(cluster_0_mean) 
        
        self.clustersave = [array / 5 for array in self.clusterdiv]
        return self.clustersave
    
    def coordrequest(self):
        if len(self.clustersave) == 0:
            self.capture_frame()
        else:
            del self.clustersave[0]  
            return self.clustersave
        
    def service_callback(self, request, response):
        self.coordrequest()
        if self.cups_detected == 0:
            response.x, response.y = 0.0, 0.0
            self.get_logger().info('No valid coordinates found.')
            return response
        if not self.clustersave:
            response.x, response.y = 0.0 , 0.0
            self.get_logger().info('No valid coordinates found.')
            return response
        if request.signal == 1:
            response.x = self.clustersave[0][0]
            response.y = self.clustersave[0][1] 
            if response.x and response.y:
                self.get_logger().info(f'Predicted coordinates: x={response.x}, y={response.y}')
                return response
            else:
                response.x, response.y = 0.0, 0.0
                self.get_logger().info('No valid coordinates found.')
                return response 
        else:  
            response.x, response.y = 0.0, 0.0
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
