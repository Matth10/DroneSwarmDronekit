""""
This script allow the copter to armed, take off and navigate using GPS location point (3) and then he returns home

"""
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math


#Set up option parsing to get connection string
import argparse

from pymavlink import mavutil

parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)

print "Launch location"
home_location = vehicle.location.global_relative_frame

def arm_and_takeoff(aTargetAltitude):
    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    #Arming the drone
    print "Arming Motors"
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)


    print "Taking Off"
    vehicle.simple_takeoff(aTargetAltitude) #Taking off to the target Altitude

    while True:
        print "Location : %s" % vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude*0.95:
            print "reached target altitude"
            break
        time.sleep(1)

def get_distance_metres(aLocation1, aLocation2):
     dlat = aLocation2.lat - aLocation1.lat
     dlong = aLocation2.lon - aLocation1.lon
     return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation.lat*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation.lon*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        aLocation.alt, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)

#Take off
arm_and_takeoff(20)

#Set default airspeed
vehicle.airspeed = 10

#Set the first point (-35.361879, 149.165155)
a_location = LocationGlobalRelative(-35.361879, 149.165155, 30)
goto_position_target_global_int(a_location)



while vehicle.mode.name == "GUIDED" :
    print "distance to target : %s" % get_distance_metres(vehicle.location.global_relative_frame, a_location)
    if get_distance_metres(vehicle.location.global_relative_frame, a_location) < 0.2:
        break

print "-----------------------------------------------------------"
print "GPS lat :%s" % vehicle.location.global_relative_frame.lat + " // GPS lat theoretic : %s" % a_location.lat
print "GPS lon :%s" % vehicle.location.global_relative_frame.lon + " // GPS lat theoretic : %s" % a_location.lon
print "GPS alt :%s" % vehicle.location.global_relative_frame.alt + " // GPS lat theoretic : %s" % a_location.alt
print "distance between : %s" % get_distance_metres(vehicle.location.global_relative_frame, a_location)
print "destination reached"
print "-------------------------------------------------------------"
time.sleep(1)

#Set the second point (-35.362216, 149.165970)
b_location = LocationGlobalRelative(-35.362216, 149.165970, 30)
goto_position_target_global_int(b_location)


while vehicle.mode.name == "GUIDED" :
    print "distance to target : %s" % get_distance_metres(vehicle.location.global_relative_frame, b_location)
    if get_distance_metres(vehicle.location.global_relative_frame, b_location) < 0.2:
        break

print "-----------------------------------------------------------"
print "GPS lat :%s" % vehicle.location.global_relative_frame.lat + " // GPS lat theoretic : %s" % b_location.lat
print "GPS lon :%s" % vehicle.location.global_relative_frame.lon + " // GPS lon theoretic : %s" % b_location.lon
print "GPS alt :%s" % vehicle.location.global_relative_frame.alt + " // GPS alt theoretic : %s" % b_location.alt
print "distance between : %s" % get_distance_metres(vehicle.location.global_relative_frame, b_location)
print "destination reached"
print "-------------------------------------------------------------"
time.sleep(1)

#Set the third point (-35.362445, 149.166203)
c_location = LocationGlobalRelative(-35.362445, 149.166203, 30)
goto_position_target_global_int(c_location)


while vehicle.mode.name == "GUIDED" :
    print "distance to target3 : %s" %get_distance_metres(vehicle.location.global_relative_frame, c_location)
    if get_distance_metres(vehicle.location.global_relative_frame, c_location) < 0.2:
        break

print "-----------------------------------------------------------"
print "GPS lat :%s" % vehicle.location.global_relative_frame.lat + " // GPS lat theoretic : %s" % c_location.lat
print "GPS lon :%s" % vehicle.location.global_relative_frame.lon + " // GPS lat theoretic : %s" % c_location.lon
print "GPS alt :%s" % vehicle.location.global_relative_frame.alt + " // GPS lat theoretic : %s" % c_location.alt
print "distance between : %s" % get_distance_metres(vehicle.location.global_relative_frame, c_location)
print "destination reached"
print "-----------------------------------------------------------"
time.sleep(1)

#Come back home
vehicle.mode = VehicleMode("RTL")
while vehicle.mode.name == "RTL":
    print "distance to target : %s" % get_distance_metres(vehicle.location.global_relative_frame, home_location)
    if get_distance_metres(vehicle.location.global_relative_frame, home_location) < 0.2:
        break

print "-----------------------------------------------------------"
print "GPS lat :%s" % vehicle.location.global_relative_frame.lat + " // GPS lat theoretic : %s" % home_location.alt
print "GPS lon :%s" % vehicle.location.global_relative_frame.lon + " // GPS lon theoretic : %s" % home_location.lon
print "GPS alt :%s" % vehicle.location.global_relative_frame.alt + " // GPS alt theoretic : %s" % home_location.alt
print "destination reached"
print "-----------------------------------------------------------"

print "landing the drone"
vehicle.mode = VehicleMode("LAND")

#disarmed the copter
vehicle.armed = False

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

#Shut down simulator
sitl.stop()
print "Completed"