import numpy as np
import cv2
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative,Vehicle, LocationGlobal, Command
import time
from time import gmtime, strftime
# vehicle = connect("/dev/serial0", wait_ready=True, baud=921000)
vehicle = connect("tcp:127.0.0.1:5762", wait_ready=True)


# def arm_and_takeoff(aTargetAltitude):
#     print("Basic pre-arm checks")
#     while not vehicle.is_armable:
#         print(" Waiting for vehicle to initialise...")
#         time.sleep(1)

#     print("Arming motors")
#     vehicle.mode = VehicleMode("GUIDED")
#     vehicle.armed = True

#     time.sleep(2)

#     print(vehicle.mode)

#     while not vehicle.armed:
#         print(" Waiting for arming...")
#         time.sleep(1)

#     print("Taking off!")
#     vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude
    
    
# arm_and_takeoff(2)













