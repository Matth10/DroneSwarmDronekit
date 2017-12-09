#Run with Simulator
print "Start simulator (STIL)"
import dronekit_sitl
stil = dronekit_sitl.start_default()
connection_string = stil.connection_string()


#Run Without a simulator
import dronekit
import socket
import exceptions
#connection_string = '/dev/ttyAMA0 (also set baud=57600)'



#Import DroneKit Python
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

#Check the connection
#try :
#    dronekit.connect(connection_string, heartbeat_timeout=15)
#    print "connection succeed"

#Bad TCP connection
#except socket.error:
#    print 'No Server exists !'

#Bad TTY connection
#except exceptions.OSError as e:
#    print 'No serial exists !'

#API Error
#except dronekit.APIException:
#    print 'Timeout!'

#Other error
#except:
#    print 'Some other error!'

#Connect and create the Vehicle
print ("Connecting to the vehicle on : %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

# Function to arm and then takeoff to a user specified altitude
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


#Take off with altitude 1
arm_and_takeoff(1)

print "landing the drone"
vehicle.mode = VehicleMode("LAND")

# Close vehicle object
vehicle.close()






