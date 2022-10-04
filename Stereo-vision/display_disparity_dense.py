# display_disparity.py 

from glob import glob
import numpy as np
import cv2
#import cv2
from stereovision.calibration import StereoCalibration
from stereovision.stereo_cameras import CalibratedPair
# from stereovision.blockmatchers import StereoBM, StereoSGBM
from stereovision.point_cloud import PointCloud
import time

# DENSE --------------------------------------------------------------------------------------------------
# Create LEFT and RIGHT blockmatchers
window_size = 5                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely 
left_matcher = cv2.StereoSGBM_create(
    minDisparity=0,
    numDisparities=160,             # max_disp has to be dividable by 16 f. E. HH 192, 256
    blockSize=5,
    P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
    P2=32 * 3 * window_size ** 2,
    disp12MaxDiff=1,
    uniquenessRatio=15,
    speckleWindowSize=0,
    speckleRange=2,
    preFilterCap=63,
    mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
)
right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
# Create filter
lmbda = 80000 # Large values of lambda force filtered disparity map edges to adhere more to source image edges 
sigma = 1.2
wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)
# END DENSE ---------------------------------------------------------------------------------------------

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
my_bm = cv2.StereoSGBM_create(minDisparity=16,
                                uniquenessRatio=10,
                                speckleWindowSize=100,
                                speckleRange=32,
                                P1=216,
                                P2=864)

cap0 = cv2.VideoCapture(0) # 0 should be LEFT
cap1 = cv2.VideoCapture(2) # 1 should be RIGHT

while True: 

    # Read stereo pair
    returnL, frameL = cap0.read()
    returnR, frameR = cap1.read()   

    # Undistort and rectify stereo pair
    im_left_remapped = cv2.remap(frameL,map1_x,map1_y,cv2.INTER_CUBIC)
    im_right_remapped = cv2.remap(frameR,map2_x,map2_y,cv2.INTER_CUBIC)

    # QUI HO CIO' CHE MI INTERESSA: Un paio rettificato -------------------------------------------------------------

    disparity_my_bm = my_bm.compute(im_left_remapped, im_right_remapped).astype(np.float32)/16
    norm_coeff = 255 / disparity_my_bm.max()
    disparity_my_bm = disparity_my_bm * norm_coeff / 255
    # Qui ho la mia mappa di disparita che mi piace 


    cv2.imshow('Disparity', disparity_my_bm)
    if cv2.waitKey(1) & 0xFF == ord('s'):
        print("You pressed a key")

        displ = left_matcher.compute(im_left_remapped, im_right_remapped).astype(np.float32)/16
        dispr = right_matcher.compute(im_right_remapped, im_left_remapped).astype(np.float32)/16
        displ = np.int16(displ)
        dispr = np.int16(dispr)
        filteredImg = wls_filter.filter(displ, im_left_remapped, None, dispr)  # important to put "imgL" here!!!
        filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
        filteredImg = np.uint8(filteredImg)

        my_rectified_pair = [im_left_remapped, im_right_remapped]
        my_calibration = StereoCalibration(input_folder="CALIBRATION_OUTPUT")
        my_disp_to_depth_mat = my_calibration.disp_to_depth_mat
        points = cv2.reprojectImageTo3D(filteredImg, my_disp_to_depth_mat, handleMissingValues=False)
        colors = cv2.cvtColor(my_rectified_pair[0], cv2.COLOR_BGR2RGB)
        obtained = PointCloud(points, colors)
        obtained = obtained.filter_infinity()
        obtained.write_ply("model_dense.ply")




        break





