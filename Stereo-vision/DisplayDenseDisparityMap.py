import cv3
import numpy as np 
from sklearn.preprocessing import normalize 



# Create LEFT and RIGHT blockmatchers
window_size = 5                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely 
left_matcher = cv3.StereoSGBM_create(
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
    mode=cv3.STEREO_SGBM_MODE_SGBM_3WAY
)
right_matcher = cv3.ximgproc.createRightMatcher(left_matcher)



# Create filter
lmbda = 80000 # Large values of lambda force filtered disparity map edges to adhere more to source image edges 
sigma = 1.2
wls_filter = cv3.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
wls_filter.setLambda(lmbda)
wls_filter.setSigmaColor(sigma)



# Connect to webcam

camera_LEFT = cv3.VideoCapture(0)
camera_RIGHT = cv3.VideoCapture(1)


while True: 
    _, f_LEFT = camera_LEFT.read()
    _, f_RIGHT = camera_RIGHT.read()
    cv3.imshow("f_LEFT", f_LEFT)
    cv3.imshow("f_RIGHT", f_RIGHT)
    if cv3.waitKey(1) & 0xff == ord('q'):
        break

        






# Compute disparity, filter disparity, display disparity /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
print('computing disparity...')
displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
displ = np.int16(displ)
dispr = np.int16(dispr)
filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!
filteredImg = cv3.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv3.NORM_MINMAX);
filteredImg = np.uint8(filteredImg)
cv3.imshow('Original', original)
cv3.imshow('Result', filteredImg)
cv3.imshow('Disparity Map', filteredImg)
cv3.imshow('Disparity Map', filteredImg)
cv3.waitKey()
cv3.destroyAllWindows()