from xxlimited import Str
from dronekit import Command,connect,VehicleMode,LocationGlobalRelative
import time
from pymavlink import mavutil
iha = connect('127.0.0.1:14550',wait_ready= True)
import cv2
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
        
           
for i in range(1,4): 
    def gorev_ekle():
        global komut, cap
        
        komut = iha.commands  
        komut.clear()
        time.sleep(1) 
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3630740, 149.1652789, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1652568, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3628596, 149.1651750, 25))
        
        cap = cv2.VideoCapture(2)
        cv2.namedWindow("Trackbars")

        cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing)
        cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
        cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

        while True:
            _, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            l_h = cv2.getTrackbarPos("L - H", "Trackbars")
            l_s = cv2.getTrackbarPos("L - S", "Trackbars")
            l_v = cv2.getTrackbarPos("L - V", "Trackbars")
            u_h = cv2.getTrackbarPos("U - H", "Trackbars")
            u_s = cv2.getTrackbarPos("U - S", "Trackbars")
            u_v = cv2.getTrackbarPos("U - V", "Trackbars")
            
            lower_blue = np.array([l_h, l_s, l_v])
            upper_blue = np.array([u_h, u_s, u_v])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
            result = cv2.bitwise_and(frame, frame, mask=mask)
            
            cv2.imshow("frame", frame)
            cv2.imshow("mask", mask)
            cv2.imshow("result", result)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            
            
        cap.release()
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