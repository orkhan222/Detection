from xxlimited import Str
from dronekit import Command,connect,VehicleMode,LocationGlobalRelative
import time
from pymavlink import mavutil
iha = connect('127.0.0.1:14550',wait_ready= True)


def takeoff(irtifa):
    while iha.is_armable is not True:
        # print('Iha arm edilebilir durumda degil.')
        time.sleep(1)
    # print('Iha arm edilebiler.')  
    iha.mode = VehicleMode('GUIDED')
    # print(str(iha.mode)+ 'moduna alindi.')
    iha.armed = True
    while iha.armed is not True:
        # print('Iha arm ediliyor...')
        time.sleep(0.5)
    # print('Iha arm edildi.')
    iha.simple_takeoff(irtifa)
    while iha.location.global_relative_frame.alt < irtifa *0.9:
        # print('Iha hedefe yukseliyor.')    
        time.sleep(0.5)

for i in range(1,3): 
    def gorev_ekle():
        global komut
        komut = iha.commands
            
        komut.clear()
        time.sleep(1)
            
        # TAKEOFF
        # komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, 0 , 0, 0, 0, -35.363074, 149.1652789, 10)) 
            
        # WAYPOINT
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3630740, 149.1652789, 25))
            
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1652568, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3628596, 149.1651750, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3629088, 149.1651106, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631691, 149.1651455, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631877, 149.1652206, 25))
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.3631527, 149.1652943, 25))

        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0 , 0, 0, 0, -35.363074, 149.1652789, 0))

        # Land
        komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0 , 0, 0, 0, -35.363074, 149.1652789, 10))  
        # RTL
        # komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0 , 0, 0, 0, -35.3630740, 149.1652789, 0))
            
        # DOGRULAMA
        # komut.add(Command(0, 0, 0,mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0 , 0, 0, 0, 0, 0 , 0))
            
        komut.upload()
        # print('Komutlar yukleniyor...')
            
    takeoff(10)
    gorev_ekle()

    komut.next = 0

    iha.mode = VehicleMode('AUTO')

    while True:
        next_waypoint = komut.next
            
        # print(f'Siradki komut {next_waypoint}')
        time.sleep(1)
            
        if next_waypoint is 8:
            # print('Gorev bitdi.')
            break
            
# print('Donguden cikildi.')








# -35.36324549 149.16523333-durdugu yer


