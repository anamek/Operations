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
no_vehicles = 2 # I dont think should be here


# Maximum number of vehicles
max_vehicles = 100

# randomizing
random.seed(36)
directions_cars = ['North', 'South', 'East', 'West']
random_directions = []
random_distances = []

# the cars spawned can not overlap

for i in range(max_vehicles):
    random_directions.append(random.choice(directions_cars))
    while(True):
        a = random.randrange(5, d_subscription - L)
        overlap = False
        for j in range(0, i):
            if random_directions[i] == random_directions[j]:
                if (abs(a - random_distances[j]) < 5):
                    overlap = True
        if not overlap:
            random_distances.append(a)
            break

class Vehicle:
    def __init__(self, idx, k=0, d0=-1, v0=20, t0=-1, t_access=None, a_acc=3, a_dec=-4, v_avg=15.63889):
        self.idx = idx
        # Maximum acceleration [m/s^2]
        self.a_max_acc = a_acc  # m/s^2
        # Maximum deceleration in emergency braking [m/s^2]
        self.a_max_dec = a_dec
        # Preferred average velocity or road average speed [m/s]
        self.v_avg = v_avg
        if k == 0:
            self.k = random_directions[idx]
        else:
            self.k = k
        if d0 == -1:
            self.d0 = random_distances[idx]
        else:
            self.d0 = d0
        self.v0 = v0
        if t0 == -1:
            self.t0 = self.d0 / self.v0
        else:
            self.t0 = t0
        self.t_access = t_access

class MILP_Model:
    def __init__(self, name="milp", vehicles=None, t_sim=0, t_gap1=1, t_gap2=7.5, v_max=30):  # Not good to have default be a list/mutable
        self.MILP = Model(name)
        self.MILP.setParam("OutputFlag", 0)
        self.vehicles = vehicles
        self.t_sim = t_sim
        if vehicles is None:
            vehicles = []
        self.no_vehicles = len(vehicles)
        self.ks = [vehicle.k for vehicle in vehicles]  # Direction
        self.v0s = [vehicle.v0 for vehicle in vehicles]
        self.d0s = [vehicle.d0 for vehicle in vehicles]
        self.t0s = [vehicle.t0 for vehicle in vehicles]
        self.a_accs = [vehicle.a_max_acc for vehicle in vehicles]
        self.a_decs = [vehicle.a_max_dec for vehicle in vehicles]

        # Safety gap (Headway h) [s] (parallel vehicles, includes safety factor)
        self.t_gap1 = t_gap1  # seconds
        # Safety gap conflicting movements [s] aka delta t travel calculation  (perpendicular vehicles)
        self.t_gap2 = t_gap2  # seconds
        # Speed limit of the road  [m/s]
        self.v_max = v_max  # m/s

        self.t = {}  # access times
        self.t_min = {}  # access times
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

        # Binary variable one per pair of vehicles (constr. 2 & 3) (+ no_vehicles*(no_vehicles-1))
        for i in range(self.no_vehicles):
            for j in range(i + 1, self.no_vehicles):
                self.B2[i, j] = self.MILP.addVar(vtype=GRB.BINARY, name="B2[%d,%d]" % (i, j))
                self.B3[i, j] = self.MILP.addVar(vtype=GRB.BINARY, name="B3[%d,%d]" % (i, j))

        self.MILP.update()

    def initialize_constraints(self):
        # Constraint 1  (+no_vehicles constraints)
        for i in range(self.no_vehicles):
            v = np.sqrt(self.v0s[i] ** 2 + 2 * self.a_accs[i] * self.d0s[i])
            dt1 = (min(self.v_max, v) - self.v0s[i]) / self.a_accs[i]
            dt2 = max(self.d0s[i] - (self.v_max ** 2 - self.v0s[i] ** 2) / (2 * self.a_accs[i]), 0) / self.v_max
            self.t_min[i] = self.t_sim + dt1 + dt2
            self.C1[i] = self.MILP.addConstr(self.t[i] >= self.t_min[i], name="C1[%d]" % i)

        # Calculated lower bound value for big M for constraint 2 and 3
        M_big = 2000

        # Constraint 2
        for j in range(self.no_vehicles):
            for k in range(j + 1, self.no_vehicles):
                if self.ks[k] == self.ks[j]:
                    pair = (j, k)
                    self.C2[pair] = self.MILP.addConstr(self.t[j] - self.t[k] + M_big * self.B2[j, k] >= self.t_gap1,
                                                        name="C2[%d,%d]" % (j, k))
                    pair = (k, j)
                    self.C2[pair] = self.MILP.addConstr(self.t[k] - self.t[j] + M_big * (1 - self.B2[j, k]) >=
                                                        self.t_gap1, name="C2[%d,%d]" % (k, j))
        self.MILP.update()
        # Constraint 3
        vert_dir = ["North", "South"]
        hor_dir = ["East", "West"]
        for j in range(self.no_vehicles):
            for k in range(j + 1, self.no_vehicles):
                if (self.ks[j] in vert_dir and self.ks[k] in hor_dir) or (self.ks[j] in hor_dir
                                                                          and self.ks[k] in vert_dir):
                    pair = (j, k)
                    self.C3[pair] = self.MILP.addConstr(self.t[j] - self.t[k] + M_big * self.B3[j, k] >= self.t_gap2,
                                                        name="C3[%d,%d]" % (j, k))
                    pair = (k, j)
                    self.C3[pair] = self.MILP.addConstr(self.t[k] - self.t[j] + M_big * (1 - self.B3[j, k]) >= self.t_gap2,
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

        BO = {}
        # Overtake Variable
        for i in range(self.no_vehicles):
            for j in range(i + 1, self.no_vehicles):
                if self.ks[i] == self.ks[j]:
                    BO[i, j] = self.MILP.addVar(vtype=GRB.BINARY, name="BO[%d,%d]" % (i, j))

        # Constraint Overtake
        for j in range(self.no_vehicles):
            for k in range(j + 1, self.no_vehicles):
                if self.ks[j] == self.ks[k]:
                    self.obj_constraints['cons_Overtake'] = self.MILP.addConstr(self.t[k] - self.t[j]
                                                        + 1000 * BO[j, k] >= 0, name="cons_over[%d,%d]" % (j, k))
        self.MILP.update()

        # First term J1
        j1 = self.t_slack["slackJ1"]
        # Second term J2
        j2 = 0
        for i in range(self.no_vehicles):
            j2 += self.t_slack[("slackJ2", i)]
        # Third term JO
        jO = 0
        for i in range(self.no_vehicles):
            for j in range(i + 1, self.no_vehicles):
                if self.ks[i] == self.ks[j]:
                    jO += BO[i, j]*0.00001

        # Define objective function to be MINIMIZED with weights w_1 and w_2
        obj = LinExpr()
        obj += w_1 * j1 + w_2 * j2 + jO

        self.MILP.setObjective(obj, GRB.MINIMIZE)
        self.MILP.update()

    def optimize(self):
        self.MILP.optimize()

    def getvariables(self, printing=False, only_t=False):
        # Get the values of all the decision variables
        solution = {}
        for i in self.MILP.getVars():
            if only_t and i.VarName.startswith('t'):
                solution[i.VarName] = i.X
            elif not only_t:
                solution[i.VarName] = i.X
        if printing:
            print(*(f"{name}: {solution[name]}" for name in solution), sep="\n")
        return solution


    def write(self, file="MILP.lp"):
        return self.MILP.write(file)

    def print(self):
        return print(self.MILP)

    def plot_access_times(self):
        solution = self.getvariables()
        t_access = [solution[name] for name in solution if name.startswith('t')]

        plotting.plot_vehicle_position(self.vehicles)
        plotting.plot_access_times(self.ks, t_access, self.t0s, signals=True)


# Generating 10 default vehicles

if __name__ == '__main__':
    list_vehicles = []
    for i in range(15):
        list_vehicles.append(Vehicle(i))

    example = MILP_Model("example", list_vehicles)
    example.initialize_variables()
    example.initialize_constraints()
    example.initialize_objective_function()
    example.optimize()
    example.plot_access_times()


# Potentially useful code for testing if the generated cars overlap?
# north_cars = []
# south_cars = []
# east_cars = []
# west_cars = []
# for i in range(100):
#     if random_directions[i] == 'North':
#         north_cars.append(random_distances[i])
#     if random_directions[i] == 'South':
#         south_cars.append(random_distances[i])
#     if random_directions[i] == 'East':
#         east_cars.append(random_distances[i])
#     if random_directions[i] == 'West':
#         west_cars.append(random_distances[i])




