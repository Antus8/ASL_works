# display_disparity.py 

from glob import glob
import numpy as np
import cv2
from stereovision.calibration import StereoCalibration
from stereovision.stereo_cameras import CalibratedPair
from stereovision.blockmatchers import StereoBM, StereoSGBM
import time

map1_x = np.load('map1_x.npy')
map1_y = np.load('map1_y.npy')
map2_x = np.load('map2_x.npy')
map2_y = np.load('map2_y.npy')

'''
my_bm = cv2.StereoSGBM( minDisparity=16,
                        numDisparities=96,
                        SADWindowSize=3,
                        uniquenessRatio=10,
                        speckleWindowSize=100,
                        speckleRange=32,
                        disp12MaxDiff=1,
                        P1=216,
                        P2=864,
                        fullDP=False)
'''
# my_bm = cv2.StereoBM_create(numDisparities=16, blockSize=15)
my_bm = cv2.StereoSGBM_create(minDisparity=16,
                        numDisparities=96,
                        uniquenessRatio=10,
                        speckleWindowSize=100,
                        speckleRange=32,
                        disp12MaxDiff=1,
                        P1=216,
                        P2=864
                        )




cap0 = cv2.VideoCapture(0) # 0 should be LEFT
cap1 = cv2.VideoCapture(2) # 1 should be RIGHT

while True: 

    # Read stereo pair
    returnL, frameL = cap0.read()
    returnR, frameR = cap1.read()   

    # Undistort and rectify stereo pair
    im_left_remapped = cv2.remap(frameL,map1_x,map1_y,cv2.INTER_CUBIC)
    im_right_remapped = cv2.remap(frameR,map2_x,map2_y,cv2.INTER_CUBIC)

    im_left_remapped = cv2.cvtColor(im_left_remapped, cv2.COLOR_BGR2GRAY)
    im_right_remapped = cv2.cvtColor(im_right_remapped, cv2.COLOR_BGR2GRAY)

    # QUI HO CIO' CHE MI INTERESSA: Un paio rettificato -------------------------------------------------------------

    disparity_my_bm = my_bm.compute(im_left_remapped, im_right_remapped).astype(np.float32)/16
    norm_coeff = 255 / disparity_my_bm.max()
    disparity_my_bm = disparity_my_bm * norm_coeff / 255
    # Qui ho la mia mappa di disparita che mi piace 


    cv2.imshow('Disparity', disparity_my_bm)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        print("You pressed a key")
        cv2.imwrite("left.ppm", im_left_remapped)
        cv2.imwrite("right.ppm", im_right_remapped)
        cv2.imwrite("disparity.ppm",disparity_my_bm) # OpenCV saves frame with RGB components 
        break
    
