import cv2
  
cap0 = cv2.VideoCapture(0)
cap1 = cv2.VideoCapture(2)
  
while(True):
    _, frame = cap0.read()
    _1, frame1 = cap1.read()
    cv2.imshow('frame', frame1)
      
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
cap0.release()
cap1.release()
cv2.destroyAllWindows()