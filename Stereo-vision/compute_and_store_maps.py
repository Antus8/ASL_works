# myRealTimeDisparity.py 

from glob import glob
import numpy as np
import cv2
#import cv3
from stereovision.calibration import StereoCalibration
from stereovision.stereo_cameras import CalibratedPair
from stereovision.blockmatchers import StereoSGBM
import time

calib_folder = "CALIBRATION_OUTPUT"
block_matcher = StereoSGBM()
camera_pair = CalibratedPair(None, StereoCalibration(input_folder=calib_folder), block_matcher)

CALIBRATION_IMAGES_PATH="CALIBRATION_IMAGES"
# Specify chessboard parameters
chessbard_rows = 15
chessboard_cols = 10
chessboard_total = chessbard_rows * chessboard_cols
x,y=np.meshgrid(range(chessbard_rows),range(chessboard_cols))
world_points=np.hstack((x.reshape(chessboard_total,1),y.reshape(chessboard_total,1),np.zeros((chessboard_total,1)))).astype(np.float32)

"""
-------------------------------------------- SINGLE CAMERA CALIBRATION ---------------------------------------------------
"""
# Process all calibration pictures taken from LEFT camera in order to understand SINGLE CAMERA CALIBRATION parameter
print("Single Camera Calibration starting...")
_3d_points=[]
_2d_points=[]

img_paths=glob(CALIBRATION_IMAGES_PATH+'/left*.ppm') #get paths of all all images
for path in img_paths:
    print("Elaborating current image: " + path)
    im=cv2.imread(path)
    ret, corners = cv2.findChessboardCorners(im, (chessbard_rows,chessboard_cols))    
    if ret: #add points only if checkerboard was correctly detected:
        _2d_points.append(corners) #append current 2D points
        _3d_points.append(world_points) #3D points are always the same

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(_3d_points, _2d_points, (im.shape[1],im.shape[0]), None, None)
# QUI HO OTTENUTO I PARAMETRI DI CALIBRAZIONE DELLA SINGOLA CAMERA (left)
print("* * * Single Camera Calibration finished")

"""
-------------------------------------------- STEREO CALIBRATION ---------------------------------------------------
"""
# Start stereo calibration stage 
print("Stereo Camera Calibration starting...")
all_right_corners=[]
all_left_corners=[]
all_3d_points=[]
idx = range(1, 51)
valid_idxs=[] 
for i in idx:
    print("Working on the current stereo pair" + str(i))
    im_left=cv2.imread(CALIBRATION_IMAGES_PATH+"/left%02d.ppm"%i)
    im_right=cv2.imread(CALIBRATION_IMAGES_PATH+"/right%02d.ppm"%i)    
    ret_left,left_corners=cv2.findChessboardCorners(im_left,(chessbard_rows,chessboard_cols))
    ret_right,right_corners=cv2.findChessboardCorners(im_right,(chessbard_rows,chessboard_cols))
    if ret_left and ret_right: 
        valid_idxs.append(i)
        all_right_corners.append(right_corners)
        all_left_corners.append(left_corners)
        all_3d_points.append(world_points)
retval, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(all_3d_points,  all_left_corners, all_right_corners,mtx,dist,mtx,dist,(im.shape[1],im.shape[0]),flags=cv2.CALIB_FIX_INTRINSIC)
print("* * * Stereo Camera Calibration finished, total reprojection error: " + str(retval))

# Connect to webcams
cap0 = cv2.VideoCapture(2) # 0 should be LEFT now 
cap1 = cv2.VideoCapture(0) # 1 should be RIGHT now 

# Prepare undistorting and rectifying map
dummyR, right_im = cap1.read()
dummyL, left_im = cap0.read()

R1=np.zeros((3,3)) #output 3x3 matrix
R2=np.zeros((3,3)) #output 3x3 matrix
P1=np.zeros((3,4)) #output 3x4 matrix
P2=np.zeros((3,4)) #output 3x4 matrix

R1, R2, P1, P2, Q,roi1,roi2=cv2.stereoRectify(mtx, #intrinsic parameters of the first camera
   dist, #distortion parameters of the first camera
   mtx, #intrinsic parameters of the second camera
   dist, #distortion parameters of the second camera
   (left_im.shape[1],left_im.shape[0]), #image dimensions
   R, #Rotation matrix between first and second cameras (returned by cv2.stereoCalibrate)
   T, #Translation vector between coordinate systems of the cameras (returned by cv2.stereoCalibrate)
   R1,R2,P1,P2) #last 4 parameters point to inizialized output variables
R1=np.array(R1) #convert output back to numpy format
R2=np.array(R2)
P1=np.array(P1)
P2=np.array(P2)
map1_x,map1_y=cv2.initUndistortRectifyMap(mtx, dist, R1, P1, (left_im.shape[1],left_im.shape[0]), 5)
map2_x,map2_y=cv2.initUndistortRectifyMap(mtx, dist, R2, P2, (left_im.shape[1],left_im.shape[0]), 5)

np.save('map1_x.npy', map1_x)
np.save('map1_y.npy', map1_y)
np.save('map2_x.npy', map2_x)
np.save('map2_y.npy', map2_y)
