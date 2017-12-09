"""
This is an implementation of  Particle swarm optimization algorithm

"""

# Importation
from __future__ import division
import random
import math
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pip._vendor.requests.packages.chardet import latin1prober
from pymavlink import mavutil
import time

from pymavlink.mavutil import location


def costFunc(aLocation1, aLocation2):
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


# class ParticleDrone

class ParticleDrone:
    def __init__(self, vehicle):
        self.position_i = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon,
                           vehicle.location.global_relative_frame.alt]  # position of the drone
        self.velocity_i = vehicle.velocity  # Drone velocity [vx,vy,vz]
        self.best_position_i = []  # Best position of the drone (individual)
        self.error_best = -1  # best error individual
        self.error = -1  # error individual

    # evaluate current fitness
    def evaluate(self):
        self.err_i = costFunc(self.position_i, self.best_position_i)

         # check to see if the current position is an individual best
        if self.err_i < self.err_best_i or self.err_best_i == -1:
            self.pos_best_i = self.position_i
            self.err_best_i = self.err_i


    def updateVelocity(self, best_position_g):
        w = 0.5  # constant inertia weight (how much to weigh the previous velocity)
        c1 = 1  # cognative constant
        c2 = 2  # social constant

        for i in range(0, 3):
            r1 = random.random()
            r2 = random.random()

            vel_cognitive = c1 * r1 * (self.best_position_i[i] - self.position_i[i])
            vel_social = c2 * r2 * (best_position_g[i] - self.position_i[i])
            self.velocity_i[i] = w * self.velocity_i[i] + vel_cognitive + vel_social


    def update_position(self, bounds):
        for i in range(0, 3):
            self.position_i[i] = self.position_i[i] + self.velocity_i[i]

            # adjust maximum position if necessary
            if self.position_i[i] > bounds[i][1]:
                self.position_i[i] = bounds[i][1]

            # adjust minimum position if neseccary
            if self.position_i[i] < bounds[i][0]:
                self.position_i[i] = bounds[i][0]


class PSO:
    def __init__(self, costFunc, x0, bounds, num_particles, maxiter, swarm):
        global num_dimensions

        num_dimensions = 3
        err_best_g = -1                   # best error for group
        pos_best_g = []                   # best position for group


        # begin optimization loop
        i=0
        while i < maxiter:                  # TODO
            #print i,err_best_g
            # cycle through particles in swarm and evaluate fitness
            for j in swarm:
                swarm[j].evaluate(costFunc)

                # determine if current particle is the best (globably)
                if swarm[j].err_i < err_best_g or err_best_g == -1:
                    pos_best_g=list(swarm[j].position_i)
                    err_best_g=float(swarm[j].err_i)

            # cycle through swarm and update velocities and position
            for j in swarm:
                swarm[j].update_velocity(pos_best_g)
                swarm[j].update_position(bounds)
            i+=1

        # print final results
        print 'FINAL:'
        print pos_best_g
        print err_best_g

