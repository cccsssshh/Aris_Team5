import cv2
import numpy as np
import os
import glob

# 비디오 캡처 초기화
capL = cv2.VideoCapture(2)
# capR = cv2.VideoCapture(4)
roi_start = (90, 240)
roi_end = (550, 480)

capL.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
capL.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# capR.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
# capR.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

folderL = 'data_final/'
# folderR = '/home/addinedu/amr_ws/aris_team5/tests/ComputerVision/StereoVisionDepthEstimation/images/stereoRight/'

num = 0
chessboardSize = (9, 6)
frameSize = (640, 480)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)  # numpy.ndarray, shape : (54, 3)
objp[:, :2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

sizeOfChessboardSquares = 20  # mm
objp = objp * sizeOfChessboardSquares

objPoints = []
imgPointsL = []
# imgPointsR = []

while True:
    checkboardFlag = False

    retL, imgL = capL.read()
    # retR, imgR = capR.read()

    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)
    # grayR = cv2.cvtColor(imgR, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, None)
    # retR, cornersR = cv2.findChessboardCorners(grayR, chessboardSize, None)


    cv2.rectangle(imgL, roi_start, roi_end, (0, 255, 0), 2)
            # 시작점과 끝점 좌표를 화면에 표시
    cv2.putText(imgL, f'Start: {roi_start}', (roi_start[0], roi_start[1] - 10), 
    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)
    cv2.putText(imgL, f'End: {roi_end}', (roi_end[0], roi_end[1] + 20), 
    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                
    cv2.namedWindow('imgL', cv2.WINDOW_FREERATIO)
    cv2.imshow('imgL', imgL)
    cv2.resizeWindow('imgL', 640, 480)
    # cv2.namedWindow('imgR', cv2.WINDOW_FREERATIO)
    # cv2.imshow('imgR', imgR)
    # cv2.resizeWindow('imgR', 640, 480)

    if retL:
        corners2L = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
        checkL = imgL.copy()
        cv2.drawChessboardCorners(checkL, chessboardSize, corners2L, retL)
        cv2.namedWindow('checkL', cv2.WINDOW_FREERATIO)
        cv2.imshow('checkL', checkL)
        cv2.resizeWindow('checkL', 640, 480)

        if cv2.waitKey(0) & 0xFF == ord('s'):
            cv2.imwrite(folderL + 'imageL' + str(num) + '.png', imgL)
            print('image saved!')
            num += 1
        else:
            print('Images not saved')

    if cv2.waitKey(5) & 0xFF == 27:  # esc키
        break

capL.release()
# capR.release()
cv2.destroyAllWindows()

# ################ FIND CHESSBOARD CORNERS - OBJECT POINTS AND IMAGE POINTS #############################

imagesLeft = sorted(glob.glob(folderL + '*.png'))
# imagesRight = sorted(glob.glob(folderR + '*.png'))

for imgLeft in imagesLeft:
    imgL = cv2.imread(imgLeft)
    grayL = cv2.cvtColor(imgL, cv2.COLOR_BGR2GRAY)

    retL, cornersL = cv2.findChessboardCorners(grayL, chessboardSize, None)

    if retL:
        objPoints.append(objp)
        cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria)
        imgPointsL.append(cornersL)
        cv2.drawChessboardCorners(imgL, chessboardSize, cornersL, retL)
        cv2.imshow('img left', imgL)
        cv2.waitKey(10)

cv2.destroyAllWindows()

# 캘리브레이션
retL, cameraMatrixL, distL, rvecsL, tvecsL = cv2.calibrateCamera(objPoints, imgPointsL, frameSize, None, None)

# undistort
imgL = cv2.imread(imagesLeft[0])  # 예시로 첫 번째 이미지를 사용합니다.
heightL, widthL, channelsL = imgL.shape
newCameraMatrixL, roiL = cv2.getOptimalNewCameraMatrix(cameraMatrixL, distL, (widthL, heightL), 1, (widthL, heightL))

dstL = cv2.undistort(imgL, cameraMatrixL, distL, None, newCameraMatrixL)

# crop the image
xL, yL, wL, hL = roiL
dstL = dstL[yL:yL + hL, xL:xL + wL]

mapxL, mapyL = cv2.initUndistortRectifyMap(cameraMatrixL, distL, None, newCameraMatrixL, (widthL, heightL), 5)
dstL = cv2.remap(imgL, mapxL, mapyL, cv2.INTER_LINEAR)

cv2.imshow('Undistorted Left', dstL)
cv2.waitKey(0)
cv2.destroyAllWindows()

print('CameraL Calibrated : ', retL)
print('CameraL Matrix : ', cameraMatrixL)
print('DistortionL Parameters : ', distL)
print('RotationL Vectors : ', rvecsL)
print('TranslationL Vectors : ', tvecsL)

# Parameters 저장
cv_file = cv2.FileStorage('calibration_params_final.xml', cv2.FILE_STORAGE_WRITE)
cv_file.write('cameraMatrixL', cameraMatrixL)
cv_file.write('distL', distL)
cv_file.write('newCameraMatrixL', newCameraMatrixL)
cv_file.write('roiL', roiL)
cv_file.release()
