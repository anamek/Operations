from MILP_OOP import MILP_Model, Vehicle

# MILP Case Study from paper
direction = ["West", "South", "East", "North", "North", "North", "West", "South", "West"]
distance = [690, 750, 780, 900, 990, 1080, 1170, 1230, 1290]
v_avg = 56.3 * 0.27778
list_vehicles = []
for i in range(9):
    list_vehicles.append(Vehicle(i, direction[i], distance[i], v_avg))

# Optimizing
example = MILP_Model("example", list_vehicles)
example.initialize_variables()
example.initialize_constraints()
example.initialize_objective_function(0.5, 0.5)
example.MILP.optimize()

# Printing results
all_vars = example.MILP.getVars()
values = example.MILP.getAttr("X",all_vars[0:9])
names = example.MILP.getAttr("VarName", all_vars[0:9])

for name, val in zip(names, values):
    print(f"{name} = {val}")




# example = MILP_Model("simple case",2, 0)
# example.vehicles[0][0].k = "North"
# example.vehicles[0][0].d0 = 1000
# example.vehicles[0][0].v0 = 20
# example.vehicles[0][0].t0 = 1000/20
#
# example.vehicles[1][0].k = "North"
# example.vehicles[1][0].d0 = 1100
# example.vehicles[1][0].v0 = 20
# example.vehicles[1][0].t0 = 1100/20
#
# example.MILP.optimize()
# solution = []
# for i in example.MILP.getVars():
#     solution.append([i.VarName, i.X])
#     print(f"{i.VarName} = {i.X}")
# print(solution)
