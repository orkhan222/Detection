import cv2
import numpy as np
import imutils

cap = cv2.VideoCapture(0)

cap.set(3,640)
cap.set(4,480)

while True:
    hsv,frame = cap.read()
    
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    lower_red = np.array([136,87,111])
    upper_red = np.array([180,255,255])
    
    mask = cv2.inRange(hsv,lower_red,upper_red)
    
    cnts = cv2.findContours(mask, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    
    for c in cnts:
        area = cv2.contourArea(c)
        if area > 500:
        
            cv2.drawContours(frame,[c],-1,(0,255,0), 3)
            
            M = cv2.moments(c)
            
            cx = int(M['m10']/ M['m00'])
            cy = int(M['m01']/ M['m00'])
            
            cv2.circle(frame,(cx,cy),7,(255,255,255), -1)
            cv2.putText(frame, 'Centre', (cx-20, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5 , (0,255,0), 1)


            cv2.imshow('frame',frame)
        
        
    print("area is ...",area)
    # print('centroid is at ...', cx,cy)
    
    key = cv2.waitKey(1000) & 0xFF
    # press 'q' to stop the loop
    if key == ord("q"):
        break
    
cap.release()
cv2.destroyAllWindows()