"""
Swarm drone follow the leader : steering behavior. Using three rules :
        arrival : following the leader and reduce the speed when you are close to him
        separation : avoid crowding
        evade : evade if you are in the leader path 

"""
# Importation
from __future__ import division
import random
import math
from locale import normalize
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, Vehicle
from pip._vendor.requests.packages.chardet import latin1prober
from pymavlink import mavutil
import time

from pymavlink.mavutil import location

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


# Get the distance (meters) between two location
def get_distance_metres(leader, follower):
    dlat = follower.location.global_relative_frame.lat - leader[0]
    dlong = follower.location.global_relative_frame.lon - leader[1]
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# get the location
def get_position(distance, target):
    coef = distance * 0.00000898311175
    new_lat = target[0] + coef
    new_long = target[1] + coef / math.cos(target[0] * 0.018)
    return new_lat, new_long

def goto_position_target_global_int(aLocation, vehicle):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified location.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111111000, # type_mask (only speeds enabled)
        aLocation[0]*1e7, # lat_int - X Position in WGS84 frame in 1e7 * meters
        aLocation[1]*1e7, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        10, # alt - Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT
        0, # X velocity in NED frame in m/s
        0, # Y velocity in NED frame in m/s
        0, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle
    vehicle.send_mavlink(msg)

# update velocity
def send_ned_velocity(velocity_x, velocity_y, velocity_z, vehicle, duration):
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

    for x in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.95)

# arm and take off the drone
def arm_and_takeoff(aTargetAltitude, vehicle):
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
    vehicle.simple_takeoff(aTargetAltitude)  # Taking off to the target Altitude

    while True:
        print "Location : %s" % vehicle.location.global_relative_frame.alt
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print "reached target altitude"
            break
        time.sleep(1)


# Normalize a vector
def normalize(velocity):
    length_velocity = math.sqrt(
        (velocity[0] * velocity[0]) + (velocity[1] * velocity[1]))
    vx_normalized = velocity[0] / length_velocity
    vy_normalized = velocity[1] / length_velocity
    return vx_normalized, vy_normalized


# Calculate the Behind Point
def behindPoint(vehicle, distance):
    assert (distance >= 1), "Distance must be superior to 1 meter"

    location_Behind = vehicle

    # Calculate the behind point
    length_velocity = math.sqrt(
        (vehicle[0] * vehicle[0]) + (vehicle[1] * vehicle[1]))

    # Normalize
    tV = vehicle
    tV = [i * -1 for i in tV]

    # if no velocity
    if length_velocity == 0:
        location_Behind.lat = get_position(distance, vehicle)[0]
        location_Behind.lon = get_position(distance, vehicle)[1]
        return location_Behind
    else:
        vx_normalized = normalize(tV)[0] * distance * 0.00000898311175
        vy_normalized = normalize(tV)[1] * distance * 0.00000898311175

        # tvlenght + position leader
        location_Behind[0] = vx_normalized + vehicle[0]
        location_Behind[1] = vy_normalized + vehicle[1]

        return location_Behind


# Calculate Ahead point
def aheadPoint(vehicle, distance):
    assert (distance >= 1), "Distance must be superior to 1 meter"

    location_Ahead = vehicle

    tV = vehicle
    vx_normalized = normalize(tV)[0] * distance * 0.00000898311175
    vy_normalized = normalize(tV)[1] * distance * 0.00000898311175

    location_Ahead[0] = vx_normalized + vehicle[0]
    location_Ahead[1] = vy_normalized + vehicle[1]

    return location_Ahead


# if the follower is on the leader path
def isOnLeaderSight(leader, leaderAhead, follower, radius):
    return get_distance_metres(leaderAhead,
                               follower) <= radius or get_distance_metres(leader, follower) <= radius


# desired velocity for seeking behavior
def desiredvelocity(follower, target):
    f_x = follower.location.global_relative_frame.lat
    f_y = follower.location.global_relative_frame.lon
    t_x = target[0]
    t_y = target[1]
    desired_velocity = []

    desired_velocity.append(t_x - f_x)
    desired_velocity.append(t_y - f_y)

    return desired_velocity


# desired velocity for fleeing behavior
def desiredvelocitybis(follower, target):
    f_x = follower.location.global_relative_frame.lat
    f_y = follower.location.global_relative_frame.lon
    t_x = target[0]
    t_y = target[1]
    desired_velocity = []

    desired_velocity.append(-t_x + f_x)
    desired_velocity.append(-t_y + f_y)

    return desired_velocity


# flee behavior
def fleebis(follower, target, max_velocity):

    # Normalized the vector
    desired_velocity = desiredvelocitybis(follower, target)
    length_velocity = math.sqrt(
        (desired_velocity[0] * desired_velocity[0]) + (desired_velocity[1] * desired_velocity[1]))
    vx_normalized = (desired_velocity[0] / length_velocity) * max_velocity
    vy_normalized = (desired_velocity[1] / length_velocity) * max_velocity
    desired_velocity = []
    desired_velocity.append(vx_normalized)
    desired_velocity.append(vy_normalized)

    # calculate the steering based on the desired_velocity
    steering_x = desired_velocity[0] - follower.velocity[0]
    steering_y = desired_velocity[1] - follower.velocity[1]

    return steering_x, steering_y


# First Rule : Following and Arriving
def arrive(follower, leader, radius, velocity_max):

    # Calculate the desired velocity
    desired_velocity = desiredvelocity(follower, leader)
    length_velocity = math.sqrt(
        (desired_velocity[0] * desired_velocity[0]) + (desired_velocity[1] * desired_velocity[1]))

    # Normalized the desired velocity
    vx_normalized = desired_velocity[0] / length_velocity
    vy_normalized = desired_velocity[1] / length_velocity
    desired_velocity = []
    desired_velocity.append(vx_normalized)
    desired_velocity.append(vy_normalized)

    # calculate the distance between the leader follower
    distance = get_distance_metres(leader, follower)

    # Check the distance whether the drone
    # inside the slowing aera
    if distance < radius:
        for i in range(0, 2):
            desired_velocity[i] *= velocity_max * (distance / radius)
    else:
        for i in range(0, 2):
            desired_velocity[i] *= velocity_max

    # calculate the steering based on the desired_velocity
    steering_x = desired_velocity[0] - follower.velocity[0]
    steering_y = desired_velocity[1] - follower.velocity[1]

    return steering_x, steering_y


# First Rule : Following and Arriving
def arrive2(follower, leader, radius, max_speed):
    # calculate the distance between the leader follower
    distance = get_distance_metres(leader, follower)

    # Check the distance whether the drone
    # inside the slowing aera
    if distance < radius:
        follower.groundspeed *= distance/radius
    else:
        follower.groundspeed = max_speed

    goto_position_target_global_int(leader)


# Second Rule : Staying out of the way
def evade(leader, follower, max_velocity):
    leader_velocity = [0.00000898311175, 0.00000898311175]
    futurPosition = leader
    distance = get_distance_metres(leader, follower)
    updatesAhead = distance / max_velocity

    # futur position of the leader
    futurPosition[0] += leader_velocity[0] * updatesAhead
    futurPosition[1] += leader_velocity[1] * updatesAhead
    return fleebis(follower, futurPosition, max_velocity)

# Third rule : Separation
def separation(swarm, follower, separation_radius):
        result_velocity = []
        neigthbor = 0

        # calculate the average of distance between each drone close to drone[i]
        for drone in swarm:
            if drone != follower & get_distance_metres(drone, follower) <= separation_radius:
                result_velocity[0] += drone.location.global_relative_frame.lat - follower.location.global_relative_frame.lat
                result_velocity[1] += drone.location.global_relative_frame.lon - follower.location.global_relative_frame.lon
                neigthbor += 1

        # If there are some neigthbors : calculate the desired velocity to roll away to the other drones
        if neigthbor != 0:
            result_velocity[0] /= neigthbor * -1
            result_velocity[1] /= neigthbor * -1

        normalize(result_velocity)
        result_velocity[0] *= separation_radius
        result_velocity[1] *= separation_radius

        send_ned_velocity(result_velocity[0], result_velocity[1], 0, follower, 1)



"""
This is the final function is combine the three rules :
    - arrive
    - evade
    - separation

It takes a leader, a follower and the swarm (all the followers)
It update the velocity of the follower
"""
def followTheLeader(leader, follower):
    radius = 1
    slowingradius = 3
    max_velocity = 10

    # calculate the behind Point
    target = behindPoint(leader, 1.5)

    # calculate ahead point
    ahead = aheadPoint(leader, radius)

    if isOnLeaderSight(leader, ahead, follower, radius):
        force = evade(leader, follower, max_velocity)
        send_ned_velocity(force[0], force[1], 0, follower, 1)

    # Create separation velocity
    # force = separation(swarm, follower, 0.70)
    # send_ned_velocity(force[0], force[1], 0, follower, 1)

    # Create velocity
    force = arrive(follower, target, slowingradius, max_velocity)
    send_ned_velocity(force[0], force[1], 0, follower, 1)








"""

This is a simulation of the function followTheLeader. In this simulation we have a follower which is a vehicle
and the leader simulated by a moving point.

"""

# a moving point that simulate the leader
def pointmovingfoward(target):
    target[0] -= 0.00000898311175
    target[1] -= 0.00000898311175


# Create the follower and leader
follower = connect(connection_string, wait_ready=True)
leader = [-35.361979, 149.165155]

# add a slowing radius
slowingradius = 1

# arm and take off the drone
arm_and_takeoff(10, follower)

print follower.location.global_relative_frame
print get_distance_metres(leader, follower)




while True:

    # get the distance between the leader and the follower in meter

    followTheLeader(leader, follower)

    pointmovingfoward(leader)

    # print leader
    # print follower.location.global_relative_frame
    print "distance left : %s" % get_distance_metres(leader, follower)
    print "groundpeed : %s" % follower.groundspeed
