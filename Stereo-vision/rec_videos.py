import cv2 

print("Application starting...")

# Get webcam
cap = cv2.VideoCapture(0)
fps = cap.get(cv2.CAP_PROP_FPS)
_, sample = cap.read()
print(sample.shape)

# Create writer
w, h = 640, 480
filename = "off_1.avi" # --------------------------------------------------> 
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = cv2.VideoWriter(filename, fourcc, fps, (w,h))

while cap.isOpened(): 
    _, f = cap.read()
    if _: 
        out.write(f)
        cv2.imshow("f", f)
    else: 
        print("ERROR: Couldn't read from webcam")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
out.release()
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

