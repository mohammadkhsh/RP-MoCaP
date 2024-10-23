import cv2

cap = cv2.VideoCapture(1) # use the camera at /dev/video0
while True:
    ret, frame = cap.read()
    cv2.imshow('frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): # press 'q' to quit
        break

cap.release()
cv2.destroyAllWindows()
