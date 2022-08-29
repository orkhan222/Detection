from xxlimited import Str
from dronekit import Command,connect,VehicleMode,LocationGlobalRelative
import time
from pymavlink import mavutil
iha = connect('127.0.0.1:14550',wait_ready= True)
import numpy as np
import argparse
# import imutils
import cv2
import urllib 
 
camera = cv2.VideoCapture( 0)

def takeoff(irtifa):
    while iha.is_armable is not True:
        time.sleep(1) 
    iha.mode = VehicleMode('GUIDED')
    iha.armed = True
    while iha.armed is not True:
        time.sleep(0.5)
    iha.simple_takeoff(irtifa)
    while iha.location.global_relative_frame.alt < irtifa *0.9:
        time.sleep(0.5)

for i in range(1,3): 
    def gorev_ekle():
        global komut
        komut = iha.commands
        komut.clear()
        time.sleep(1)             
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1652568, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3628596, 149.1651750, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1651106, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3630433, 149.1652058, 10))
        from collections import deque

 
        ap = argparse.ArgumentParser()

        args = vars(ap.parse_args())
        
        lower = {'red':(136, 87, 111),'blue':(97, 100, 117)} 

        upper = {'red':(180,255,255),'blue':(117,255,255)}
        
        colors = {'red':(0,0,255),'blue':(87,152,250)}
        


        while True:

            (grabbed, frame) = camera.read()
            if args.get("video") and not grabbed:
                break
        
        
            # frame = imutils.resize(frame, width=900)
        
            blurred = cv2.GaussianBlur(frame, (11, 11), 0)
            hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

            for key, value in upper.items():
            
                kernel = np.ones((9,9),np.uint8)
                mask = cv2.inRange(hsv, lower[key], upper[key])
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
                    
            
                cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
                    cv2.CHAIN_APPROX_SIMPLE)[-2]
                center = None
            


                if len(cnts) > 0:
                    
                    c = max(cnts, key=cv2.contourArea)
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    M = cv2.moments(c)
                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            

                    if radius > 50:
                    
                        cv2.circle(frame, (int(x), int(y)), int(radius), colors[key], 5)
                        cv2.putText(frame,key + "color", (int(x-radius),int(y-radius)),cv2.FONT_HERSHEY_SIMPLEX, 0.6,colors[key],2)

            cv2.imshow("Frame", frame)
        
            key = cv2.waitKey(1) & 0xFF
            # press 'q' to stop the loop
            if key == ord("q"):
                break
# print(col)
        
        cv2.destroyAllWindows()
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631527, 149.1652943, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631877, 149.1652206, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631691, 149.1651455, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3630433, 149.1652058, 10))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0 , 0, 0, 0, -35.3630740, 149.1652789, 0))  
        komut.upload()
    takeoff(10)
    camera.release()
    gorev_ekle()
    komut.next = 0
    iha.mode = VehicleMode('AUTO')
    while True:
        next_waypoint = komut.next
        time.sleep(1)
        if next_waypoint is 8:
            
            break
        