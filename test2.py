from xxlimited import Str
from dronekit import Command,connect,VehicleMode,LocationGlobalRelative
import time
from pymavlink import mavutil
iha = connect('127.0.0.1:14550',wait_ready= True)
import cv2
from time import gmtime, strftime 
import numpy as np

def nothing(x):
    pass




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
        
cap = cv2.VideoCapture(2)
fourcc = cv2.VideoWriter_fourcc(*'XVID')
file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))

           
for i in range(1,4): 
    def gorev_ekle():
        global komut, cap
        
        komut = iha.commands  
        komut.clear()
        time.sleep(1) 
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3630740, 149.1652789, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1652568, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3628596, 149.1651750, 25))
        
        
        # cap = cv2.VideoCapture(2)
        # fourcc = cv2.VideoWriter_fourcc(*'XVID')
        # file_name = strftime("%Y-%m-%d_%H-%M-%S", gmtime()) + ".avi"
        # out = cv2.VideoWriter(file_name,fourcc, 25, (640,480))
            
        while True: 
            ret, frame = cap.read()
            out.write(frame)
            if ret == True:
                # Filter red color
                cv2.imshow("frame",frame)
                frame = cv2.bilateralFilter(frame,9,75,75)
                frame_hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
                mask1 = cv2.inRange(frame_hsv, (0, 70, 50), (10, 255, 255))
                mask2 = cv2.inRange(frame_hsv, (165, 70, 50), (180, 255, 255))
                mask = mask1 + mask2
                white_pixels = np.where(mask==255)
                cX = np.average(white_pixels[1])
                cY = np.average(white_pixels[0])
                
                # Small noise elimination
                if len(white_pixels[0]) > 5000:
                    # Object location detection
                    img = np.zeros((480,640,1),np.uint8)    
                    cv2.circle(img, (int(cX),int(cY)), 85, (255,255,255), thickness=-1, lineType=8, shift=0)
                    intersection = cv2.bitwise_and(img,mask)
                    intersection_length = np.where(intersection==255)

                    if len(intersection_length[0]) > 5000:
                        intersection_cX= np.average(intersection_length[1])
                        intersection_cY= np.average(intersection_length[0])
                        cv2.imshow("intersection", intersection)
                        x = intersection_cX-320
                        y = 240-intersection_cY

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break


                cap.release()
                out.release()
                cv2.destroyAllWindows()
            
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1651106, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631691, 149.1651455, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631877, 149.1652206, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631527, 149.1652943, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.363074, 149.1652789, 0))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0 , 0, 0, 0, -35.363074, 149.1652789, 10))  
        komut.upload()
    takeoff(10)

    gorev_ekle()
    komut.next = 0
    iha.mode = VehicleMode('AUTO')
    while True:
        next_waypoint = komut.next
        time.sleep(1)   
        if next_waypoint is 8:
            break