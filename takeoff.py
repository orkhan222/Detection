from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

vehicle = connect(args.connect, baud=57600, wait_ready=True)

def arm_and_takeoff(aTargetAltitude):
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    time.sleep(1)

  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt )
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

# Initialize the takeoff sequence to 20m
arm_and_takeoff(2)

print("Take off complete")

# Hover for 10 seconds
time.sleep(1)

print("Now let's land")
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()