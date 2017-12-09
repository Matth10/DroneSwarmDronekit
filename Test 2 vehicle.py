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
    sitl2 = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
    connection_string2 = sitl2.connection_string()
    print connection_string
    print connection_string2

# Connect to the Vehicle

leader = connect(connection_string2, wait_ready=True)
#follower = connect(connection_string2, wait_ready=True)
