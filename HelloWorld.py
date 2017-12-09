print "Start simulator (STIL)"
import dronekit_sitl
stil = dronekit_sitl.start_default()
connection_string = stil.connection_string()

#Import DroneKit Python
from dronekit import connect, VehicleMode

#Connect to the Vehicle
print ("Connecting to the vehicle on : %s" % (connection_string,))
vehicle = connect(connection_string, wait_ready=True)

#Get some vehicle State
print "Get some vehicle attribute values :"
print "GPS : %s" % vehicle.gps_0
print "GPS lat :%s" % vehicle.location.global_relative_frame.lat
print "GPS lon :%s" % vehicle.location.global_relative_frame.lon
print "GPS alt :%s" % vehicle.location.global_relative_frame.alt
print "Battery : %s" % vehicle.battery
print "Last Heartbeat : %s" % vehicle.last_heartbeat
print "Is Armable ? %s" % vehicle.is_armable
print "System status : %s" % vehicle.system_status
print "Mode : %s" % vehicle.mode.name

#Close the vehicle object before exiting script
vehicle.close()

#Shut down simulator
stil.stop()
print("Completed")
