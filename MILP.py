import numpy as np
import pandas as pd
import os
import random
from itertools import product
from gurobipy import Model,GRB,LinExpr,quicksum
from operator import itemgetter
import matplotlib.pyplot as plt

# Defining initial values
no_vehicles = 10 # Number of vehicles
t_gap2 = 7.5 # seconds
M_big = 2000 # for constraint 3


MILP = Model("milp")


#######################################
### Defining properties of vehicles ###
#######################################

K = {k:{'Direction':random.randint(0,1)}
     for k in list(np.arange(0,no_vehicles))}  # 0 = vertical 1 = horizontal

#######################################
##### Defining decision variables #####
#######################################
t = {} # access times
B = {} # Binary vars for constraint 3

# Access times one per vehicle
for i in range(no_vehicles):
    t[i] = MILP.addVar(lb = 0.0, vtype = GRB.CONTINUOUS, name = "t")

# Binary variable one per pair of vehicles (constr. 3)
for i in range(no_vehicles):
    for j in range(i+1, no_vehicles):
        B[i, j] = MILP.addVar(vtype = GRB.BINARY, name = "B")

MILP.update()

#######################################
############ Constraints ##############
#######################################
# Constraint 3
C3 = {}
for j in range(no_vehicles):
    for k in range(j + 1, no_vehicles):
        if K[j]["Direction"] != K[k]["Direction"]:
            pair = (j, k)
            C3[pair] = MILP.addConstr(t[j] - t[k] + M_big * B[j,k] >= t_gap2, name = "C3[%d,%d]"%(j,k))
            pair = (k, j)
            C3[pair] = MILP.addConstr(t[k] - t[j] + M_big * B[j,k] >= t_gap2, name = "C3[%d,%d]"%(k,j))

MILP.update()

print(MILP)