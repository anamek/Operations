import numpy as np
import pandas as pd
import os
import random
from itertools import product
from gurobipy import Model, GRB, LinExpr, quicksum
from operator import itemgetter
import matplotlib.pyplot as plt


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


#######################################
### Defining properties of vehicles ###
#######################################
random.seed(42)

v0 = np.ones(no_vehicles) * v_avg
d0 = np.ones(no_vehicles) * 100
t0 = d0/v0  # t0 = d0/v0


v0 = [20, 20]
d0 = [1000, 1050]
t0 = [50, 50.5]
t_sim = 0



K = {k: {'Direction': random.choice(['North', 'South', 'East', 'West'])}
     for k in list(np.arange(0, no_vehicles))}  # 0 = vertical 1 = horizontal

K = {0: {"Direction": "South"},
     1: {"Direction": "North"}}
##############################
### Defining Model classes ###
##############################

class Vehicle:
    def __init__(self, idx, k=0, d0=0, v0=0, t0=0, t_access=None):
        self.idx = idx
        self.k = k
        self.d0 = d0
        self.v0 = v0
        self.t0 = t0
        self.t_access = t_access


class MILP_Model:
    def __init__(self, name="milp", no_vehicles=10, t_sim=0):
        self.MILP = Model(name)
        self.no_vehicles = no_vehicles
        self.vehicles = [[Vehicle(h, K[h]["Direction"], d0[h], v0[h], t0[h])] for h in range(no_vehicles)]
        self.t_sim = t_sim

    def initialize_variables(self):
        pass

    def initialize_constraints(self):
        pass

    def initialize_objective_function(self):
        pass

    def print(self):
        return print(self.MILP)





MILP = Model("milp")
MILP.update()

#######################################
##### Defining decision variables #####
#######################################
t = {}  # access times
B2 = {}  # Binary vars for constraint 2
B3 = {}  # Binary vars for constraint 3

# Access times one per vehicle (+no_vehicles)
for i in range(no_vehicles):
    t[i] = MILP.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name="t[%d]" % i)

# Binary variable one per pair of vehicles (constr. 2 & 3) (+ no_vehicles*(no_vehicles+1))
for i in range(no_vehicles):
    for j in range(i + 1, no_vehicles):
        B2[i, j] = MILP.addVar(vtype=GRB.BINARY, name="B2[%d,%d]" % (i, j))
        B3[i, j] = MILP.addVar(vtype=GRB.BINARY, name="B3[%d,%d]" % (i, j))

MILP.update()

#######################################
############ Constraints ##############
#######################################


C1 = {}
# Constraint 1
for k in range(no_vehicles):
    v = np.sqrt(v0[k] ** 2 + 2 * a_max_acc * d0[k])
    dt1 = (min(v_max, v) - v0[k]) / a_max_acc
    dt2 = max(d0[k] - (v_max ** 2 - v0[k] ** 2) / (2 * a_max_acc), 0) / v_max
    t_min = t_sim + dt1 + dt2
    C1[k] = MILP.addConstr(t[k] >= t_min, name="C2[%d]" % k)


# Constraint 2
C2 = {}
for j in range(no_vehicles):
    for k in range(j + 1, no_vehicles):
        if K[j]["Direction"] == K[k]["Direction"]:
            pair = (j, k)
            C2[pair] = MILP.addConstr(t[j] - t[k] + M_big * B2[j, k] >= t_gap1, name="C2[%d,%d]" % (j, k))
            pair = (k, j)
            C2[pair] = MILP.addConstr(t[k] - t[j] + M_big * (1 -B2[j, k]) >= t_gap1, name="C2[%d,%d]" % (k, j))

MILP.update()

# Constraint 3
C3 = {}
vert_dir = ["North", "South"]
hor_dir = ["East", "West"]
for j in range(no_vehicles):
    for k in range(j + 1, no_vehicles):
        if (K[j]["Direction"] in vert_dir and K[k]["Direction"] in hor_dir) or (K[j]["Direction"] in hor_dir
                                                                                and K[k]["Direction"] in vert_dir):
            pair = (j, k)
            C3[pair] = MILP.addConstr(t[j] - t[k] + M_big * B3[j, k] >= t_gap2, name="C3[%d,%d]" % (j, k))
            pair = (k, j)
            C3[pair] = MILP.addConstr(t[k] - t[j] + M_big * (1 - B3[j, k]) >= t_gap2, name="C3[%d,%d]" % (k, j))

MILP.update()


###################################
### Defining objective function ###
###################################

# Variables and Constraints for objective functions
t_slack = {}
obj_constraints = {}

# Adding J1 slack variable (+1 variable)
t_slack["slackJ1"] = MILP.addVar(lb=0.0,
                                 vtype=GRB.CONTINUOUS,
                                 name="slack_delta_t_access")

# Adding J1 constraints (+no_vehicles constraints)
for i in range(no_vehicles):
    obj_constraints[("constraintsJ1", i)] = MILP.addConstr(t_slack["slackJ1"] >= t[i],
                                                           name="cons_t_access[%d]" % i)

MILP.update()

# Adding J2 slack variables (+no_vehicles variables)
for i in range(no_vehicles):
    t_slack[("slackJ2", i)] = MILP.addVar(lb=0.0,
                                          vtype=GRB.CONTINUOUS,
                                          name="slack_delta_t_access_abs[%d]" % i)

# Adding J2 Constraints (+2*no_vehicles constraints)
for i in range(no_vehicles):
    j = 0
    obj_constraints[("constraintsJ2", i)] = MILP.addConstr(t_slack[("slackJ2", i)] >= (t[i] - t0[i]),
                                                           name="cons_t_access_pos_difference[%d]" % i)
    obj_constraints[("constraintsJ2", i + j)] = MILP.addConstr(t_slack[("slackJ2", i)] >= -(t[i] - t0[i]),
                                                               name="cons_t_access_neg_difference[%d]" % i)
    j += 1

MILP.update()

# First term J1
J1 = t_slack["slackJ1"]

# Second term J2
J2 = 0
for i in range(no_vehicles):
    J2 += t_slack[("slackJ2", i)]

# Define objective function to be MINIMIZED with weights w_1 and w_2
obj = LinExpr()
w_1 = 0
w_2 = 1
obj += w_1 * J1 + w_2 * J2
MILP.setObjective(obj, GRB.MINIMIZE)

# Update and print model
MILP.update()
# MILP.write('MILP.lp')

MILP.write('MILP.lp')
MILP.optimize()


solution = []
for i in MILP.getVars():
    solution.append([i.VarName, i.X])
    print(f"{i.VarName} = {i.X}")
print(MILP)




