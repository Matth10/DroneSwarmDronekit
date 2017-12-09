# coding=utf-8
"""
Doing a square path using velocity control

"""
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    print "Basic pre-arm checks"
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    # Arming the drone
    print "Arming Motors"
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)


    print "Taking Off"
    vehicle.simple_takeoff(aTargetAltitude) # Taking off to the target Altitude

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

def condition_yaw(heading, relative=False):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting
    the yaw using this function there is no way to return to the default yaw "follow direction
    of travel" behaviour (https://github.com/diydrones/ardupilot/issues/2427)

    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)


def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only
    velocity components .

    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version
    (sending the message multiple times does not cause problems).

    See the above link for information on the type_mask (0=enable, 1=ignore).
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


# Arm and take of to altitude of 1.5 meters
arm_and_takeoff(1.5)

print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
DURATION = 2 #Set duration for each segment.

# Set up velocity vector to map to each direction.
# vx > 0 => fly North
# vx < 0 => fly South
NORTH = 1
SOUTH = -1

# Note for vy:
# vy > 0 => fly East
# vy < 0 => fly West
EAST = 1
WEST = -1

# Note for vz:
# vz < 0 => ascend
# vz > 0 => descend
UP = -0.5
DOWN = 0.5

# Direction North
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# Yaw 90°
print"doing a rotation of 90°"
condition_yaw(90)

# Direction East
send_ned_velocity(0, EAST, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# Yaw 90°
print "Doing a rotation of 90°"
condition_yaw(90)

# Direction South
send_ned_velocity(SOUTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# Yaw 90°
print "Doing a rotation of 90°"
condition_yaw(90)

# Direction North
send_ned_velocity(NORTH, 0, 0, DURATION)
send_ned_velocity(0, 0, 0, 1)

# land the drone
print "landing the drone"
vehicle.mode = VehicleMode("LAND")


# Close vehicle object
vehicle.close()

