import numpy as np
import pandas as pd
import os
import random
from itertools import product
from gurobipy import Model, GRB, LinExpr, quicksum
from operator import itemgetter
import matplotlib.pyplot as plt
import plotting

# usage of no_vehicles vs self.no_vehicles

#####################################
### Defining all initial settings ###
#####################################
# Subscription distance [m]
d_subscription = 500
# Length of the vehicle [m]
L = 5
# Intersection square width [m]
W = 10
# Response time of an autonomous vehicle [s]
t_res = 0.5
# Number of connected vehicles
no_vehicles = 2
# Safety gap (Headway h) [s] (parallel vehicles, includes safety factor)
t_gap1 = 1  # seconds
# Safety gap conflicting movements [s] aka delta t travel calculation  (perpendicular vehicles)
t_gap2 = 7.5  # seconds
# Maximum acceleration [m/s^2]
a_max_acc = 3  # m/s^2
# Maximum deceleration in emergency braking [m/s^2]
a_max_dec = -4
# Speed limit of the road  [m/s]
v_max = 30  # m/s
# Preferred average velocity or road average speed [m/s]
v_avg = 15.63889
# Calculated lower bound value for big M for constraint 3
M_big = 2000  # for constraint 3

# randomizing
random.seed(42)
directions_cars = ['North', 'South', 'East', 'West']
random_directions = []
random_distances = []
for i in range(100):
    random_directions.append(random.choice(directions_cars))
    random_distances.append(random.randint(0, 500))


class Vehicle:
    def __init__(self, idx, k=0, d0=-1, v0=20, t0=-1, t_access=None):
        self.idx = idx
        if k == 0:
            self.k = random_directions[idx % 100]
        else:
            self.k = k
        if d0 == -1:
            self.d0 = random_distances[idx % 100]
        else:
            self.d0 = d0
        self.v0 = v0
        if t0 == -1:
            self.t0 = d0 / v0
        else:
            self.t0 = t0
        self.t_access = t_access


class MILP_Model:
    def __init__(self, name="milp", vehicles=[], t_sim=0):
        self.MILP = Model(name)
        self.no_vehicles = no_vehicles
        self.vehicles = vehicles
        self.t_sim = t_sim
        self.no_vehicles = len(vehicles)
        self.ks = [vehicle.k for vehicle in vehicles]  # Direction
        self.v0s = [vehicle.v0 for vehicle in vehicles]
        self.d0s = [vehicle.d0 for vehicle in vehicles]
        self.t0s = [vehicle.t0 for vehicle in vehicles]

        self.t = {}  # access times
        self.B2 = {}  # Binary vars for constraint 2
        self.B3 = {}  # Binary vars for constraint 3

        self.C1 = {}  # Constraint one from the paper
        self.C2 = {}  # Constraint two from the paper
        self.C3 = {}  # Constraint three from the paper
        self.t_slack = {}
        self.obj_constraints = {}

    def initialize_variables(self):
        # Access times one per vehicle (+no_vehicles)
        for i in range(self.no_vehicles):
            self.t[i] = self.MILP.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name="t[%d]" % i)

        # Binary variable one per pair of vehicles (constr. 2 & 3) (+ no_vehicles*(no_vehicles+1))
        for i in range(self.no_vehicles):
            for j in range(i + 1, self.no_vehicles):
                self.B2[i, j] = self.MILP.addVar(vtype=GRB.BINARY, name="B2[%d,%d]" % (i, j))
                self.B3[i, j] = self.MILP.addVar(vtype=GRB.BINARY, name="B3[%d,%d]" % (i, j))

        self.MILP.update()

    def initialize_constraints(self):
        # Constraint 1
        for i in range(self.no_vehicles):
            v = np.sqrt(self.v0s[i] ** 2 + 2 * a_max_acc * self.d0s[i])
            dt1 = (min(v_max, v) - self.v0s[i]) / a_max_acc
            dt2 = max(self.d0s[i] - (v_max ** 2 - self.v0s[i] ** 2) / (2 * a_max_acc), 0) / v_max
            t_min = self.t_sim + dt1 + dt2
            self.C1[i] = self.MILP.addConstr(self.t[i] >= t_min, name="C2[%d]" % i)

        # Constraint 2
        for j in range(self.no_vehicles):
            for k in range(j + 1, self.no_vehicles):
                if self.ks[k] == self.ks[j]:
                    pair = (j, k)
                    self.C2[pair] = self.MILP.addConstr(self.t[j] - self.t[k] + M_big * self.B2[j, k] >= t_gap1,
                                                        name="C2[%d,%d]" % (j, k))
                    pair = (k, j)
                    self.C2[pair] = self.MILP.addConstr(self.t[k] - self.t[j] + M_big * (1 - self.B2[j, k]) >= t_gap1,
                                                        name="C2[%d,%d]" % (k, j))
        self.MILP.update()
        # Constraint 3
        vert_dir = ["North", "South"]
        hor_dir = ["East", "West"]
        for j in range(self.no_vehicles):
            for k in range(j + 1, self.no_vehicles):
                if (self.ks[j] in vert_dir and self.ks[k] in hor_dir) or (self.ks[j] in hor_dir
                                                                          and self.ks[k] in vert_dir):
                    pair = (j, k)
                    self.C3[pair] = self.MILP.addConstr(self.t[j] - self.t[k] + M_big * self.B3[j, k] >= t_gap2,
                                                        name="C3[%d,%d]" % (j, k))
                    pair = (k, j)
                    self.C3[pair] = self.MILP.addConstr(self.t[k] - self.t[j] + M_big * (1 - self.B3[j, k]) >= t_gap2,
                                                        name="C3[%d,%d]" % (k, j))

        self.MILP.update()

    def initialize_objective_function(self, w_1=0.5, w_2=0.5):
        # Adding J1 slack variable (+1 variable)
        self.t_slack["slackJ1"] = self.MILP.addVar(lb=0.0,
                                                   vtype=GRB.CONTINUOUS,
                                                   name="slack_delta_t_access")

        # Adding J1 constraints (+no_vehicles constraints)
        for i in range(self.no_vehicles):
            self.obj_constraints[("constraintsJ1", i)] = self.MILP.addConstr(self.t_slack["slackJ1"] >= self.t[i],
                                                                             name="cons_t_access[%d]" % i)

        # Adding J2 slack variables (+no_vehicles variables)
        for i in range(self.no_vehicles):
            self.t_slack[("slackJ2", i)] = self.MILP.addVar(lb=0.0,
                                                            vtype=GRB.CONTINUOUS,
                                                            name="slack_delta_t_access_abs[%d]" % i)

        # Adding J2 Constraints (+2*no_vehicles constraints)
        for i in range(self.no_vehicles):
            j = 0
            self.obj_constraints[("constraintsJ2", i)] = \
                self.MILP.addConstr(self.t_slack[("slackJ2", i)] >= (self.t[i] - self.t0s[i]),
                                    name="cons_t_access_pos_difference[%d]" % i)

            self.obj_constraints[("constraintsJ2", i + j)] = self.MILP.addConstr(
                self.t_slack[("slackJ2", i)] >= -(self.t[i] - self.t0s[i]),
                name="cons_t_access_neg_difference[%d]" % i)
            j += 1

        # First term J1
        j1 = self.t_slack["slackJ1"]
        # Second term J2
        j2 = 0
        for i in range(self.no_vehicles):
            j2 += self.t_slack[("slackJ2", i)]

        # Define objective function to be MINIMIZED with weights w_1 and w_2
        obj = LinExpr()
        obj += w_1 * j1 + w_2 * j2

        self.MILP.setObjective(obj, GRB.MINIMIZE)
        self.MILP.update()

    def optimize(self):
        return self.MILP.optimize()

    def getvariables(self):
        # Get the values of all the decision variables
        solution = []
        for i in self.MILP.getVars():
            solution.append([i.VarName, i.X])
        return solution


    def write(self, file="MILP.lp"):
        return self.MILP.write(file)

    def print(self):
        return print(self.MILP)


# Generating 10 default vehicles


list_vehicles = []
for i in range(10):
    list_vehicles.append(Vehicle(i))


example = MILP_Model("example", list_vehicles)
example.initialize_variables()
example.initialize_constraints()
example.initialize_objective_function()
print(example.d0s) # or example.print()
# print("Directions: ", example.ks)

