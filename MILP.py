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


MILP = Model("milp")


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

print(MILP)
