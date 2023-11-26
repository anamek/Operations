import numpy as np
import pandas as pd
import os
import random
from itertools import product
from gurobipy import Model, GRB, LinExpr, quicksum
from operator import itemgetter
import matplotlib.pyplot as plt

# Comments for improvement
# Cars need to have t0, d0 and v0 for each one


#####################################
### Defining all initial settings ###
#####################################

# Subscription distance [m]
d_subscription = 500
# Number of connected vehicles
n_CV = 0 # PLACEHOLDER
# Length of the vehicle [m]
L = 5
# Intersection square width [m]
W = 10
# Response time of an autonomous vehicle [s]
t_res = 0.5
# Maximum deceleration in emergency braking [m/s^2]
a_max_dec = -4
# Preferred average velocity or raod average speed [m/s]
v_avg = 15.63889
# Speed limit of the road
v_max = 0 # PLACEHOLDER
# Maximum acceleration
a_max_acc = 3

# Safety gap (Headway h ) which includes safety factor [s] (for vehicles heading in the same direction)
t_gap1 = 1

# Safety gap conflicting movements and delta t travel calculation  (For vehicles crossing the intersection)
t_gap2 , delta_t_travel = 7.5, 7.5


class MILP_Model:
    def __init__(self, name="milp", no_vehicles=10):
        self.model = Model(name)
        self.no_vehicles = no_vehicles












# Defining initial values
no_vehicles = 10  # Number of vehicles
t_gap1 = 1  # seconds
t_gap2 = 7.5  # seconds
a_max = 3  # m/s^2
v_max = 30  # m/s
M_big = 2000  # for constraint 3

t0 = np.zeros(no_vehicles)
v0 = np.ones(no_vehicles)*20
d0 = np.ones(no_vehicles)*100


random.seed(42)
MILP = Model("milp")

t_desired = np.zeros(no_vehicles) # all initial times initialized to 0 for now


#######################################
### Defining properties of vehicles ###
#######################################

K = {k: {'Direction': random.randint(0, 1)}
     for k in list(np.arange(0, no_vehicles))}  # 0 = vertical 1 = horizontal

#######################################
##### Defining decision variables #####
#######################################
t = {}  # access times
B2 = {}  # Binary vars for constraint 2
B3 = {}  # Binary vars for constraint 3

# Access times one per vehicle
for i in range(no_vehicles):
    t[i] = MILP.addVar(lb=0.0, vtype=GRB.CONTINUOUS, name="t")

# Binary variable one per pair of vehicles (constr. 3)
for i in range(no_vehicles):
    for j in range(i+1, no_vehicles):
        B2[i, j] = MILP.addVar(vtype=GRB.BINARY, name="B2")
        B3[i, j] = MILP.addVar(vtype=GRB.BINARY, name="B3")

MILP.update()

#######################################
############ Constraints ##############
#######################################


C1 = {}
# Constraint 1
for k in range(no_vehicles):
    v = np.sqrt(v0[k] ** 2 + 2 * a_max * d0[k])
    dt1 = (min(v_max, v) - v0[k]) / a_max
    dt2 = max(d0[k] - (v_max ** 2 - v0[k] ** 2) / (2 * a_max), 0) / v_max
    t_min = t0[k] + dt1 + dt2
    C1[k] = MILP.addConstr(t[k] >= t_min, name="C2[%d]" % k)


# Constraint 2
C2 = {}
for j in range(no_vehicles):
    for k in range(j + 1, no_vehicles):
        if K[j]["Direction"] == K[k]["Direction"]:
            pair = (j, k)
            C2[pair] = MILP.addConstr(t[j] - t[k] + M_big * B2[j, k] >= t_gap1, name="C2[%d,%d]" % (j, k))
            pair = (k, j)
            C2[pair] = MILP.addConstr(t[k] - t[j] + M_big * B2[j, k] >= t_gap1, name="C2[%d,%d]" % (k, j))

MILP.update()

# Constraint 3
C3 = {}
for j in range(no_vehicles):
    for k in range(j + 1, no_vehicles):
        if K[j]["Direction"] != K[k]["Direction"]:
            pair = (j, k)
            C3[pair] = MILP.addConstr(t[j] - t[k] + M_big * B3[j, k] >= t_gap2, name="C3[%d,%d]" % (j, k))
            pair = (k, j)
            C3[pair] = MILP.addConstr(t[k] - t[j] + M_big * B3[j, k] >= t_gap2, name="C3[%d,%d]" % (k, j))

MILP.update()


##########################
### Defining objective ###
##########################

# Minimize cost function J = w_1 * J1 + w_2 * J2
obj = LinExpr()
w_1 = 0.5
w_2 = 0.5

# Variables and Constraints for objective functions
t_slack = {}
obj_constraints = {}

# Adding J1 slack variable (+1 variables)
t_slack["slackJ1"] = MILP.addVar(lb=0.0,
                                 vtype=GRB.CONTINUOUS,
                                 name="slack_delta_t_access")

# Adding J1 constraints (+no_vehicles constraints)
for i in range(no_vehicles):
    obj_constraints[("constraintsJ1", i)] = MILP.addConstr(t_slack["slackJ1"] >= t[i],
                                                           name="cons_t_access[%d]"%i)

# Adding J2 slack variables (+no_vehicles variables)
for i in range(no_vehicles):
    t_slack[("slackJ2", i)] = MILP.addVar(lb=0.0,
                                          vtype=GRB.CONTINUOUS,
                                          name="slack_delta_t_access_abs[%d]"%i)

# Adding J2 Constraints (+2*no_vehicles constraints)
for i in range(no_vehicles):
    j = 0
    obj_constraints[("constraintsJ2", i)] = MILP.addConstr(t_slack[("slackJ2", i)] >= (t[i] - t_desired[i]),
                                                           name="cons_t_access_pos_difference[%d]"%i)
    obj_constraints[("constraintsJ2", i+j)] = MILP.addConstr(t_slack[("slackJ2", i)] >= -(t[i] - t_desired[i]),
                                                             name="cons_t_access_neg_difference[%d]"%i)
    j += 1


# First term J1
J1 = t_slack["slackJ1"]

#Second term J2
J2 = 0
for i in range(no_vehicles):
    J2 += t_slack[("slackJ2", i)]

obj += w_1 * J1 + w_2 * J2


MILP.setObjective(obj, GRB.MINIMIZE)


MILP.update()
#MILP.write('MILP.lp')

print(MILP)