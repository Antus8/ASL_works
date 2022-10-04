import cv2 

print("Application starting...")

# Get webcams
capL = cv2.VideoCapture(0)
capR = cv2.VideoCapture(2)
fpsL = capL.get(cv2.CAP_PROP_FPS)
fpsR = capR.get(cv2.CAP_PROP_FPS)
if fpsL!=fpsR:
    print("ALERT: fpsL!=fpsR")
_, sample = capL.read()


# Create writer
w, h = 640, 480
filenameL = "L1.avi" 
filenameR = "R1.avi" 

fourcc = cv2.VideoWriter_fourcc(*'XVID')

outL = cv2.VideoWriter(filenameL, fourcc, fpsL, (w,h))
outR = cv2.VideoWriter(filenameR, fourcc, fpsR, (w,h))

while capL.isOpened(): 
    _L, fL = capL.read()
    _R, fR = capR.read()
    if _L and _R: 
        outL.write(fL)
        outR.write(fR)
        cv2.imshow("fL", fL)
        cv2.imshow("fR", fR)
    else: 
        print("ERROR: Couldn't read from webcam")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

capL.release()
capR.release()
outL.release()
outR.release()

cv2.destroyAllWindows()

print("Application finished")

"""
________________
________________
________________
________________
________________
________________
"""

