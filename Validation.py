from MILP_OOP import MILP_Model, Vehicle
import plotting

# MILP Case Study from paper
direction = ["West", "South", "East", "North", "North", "North", "West", "South", "West"]
distance = [690, 750, 780, 900, 990, 1080, 1170, 1230, 1290]
v_avg = 56.3 * 0.27778
v_max = 72.4 * 0.27778
list_vehicles = []
for i in range(9):
    list_vehicles.append(Vehicle(i, direction[i], distance[i], v_avg))

# Optimizing
example = MILP_Model("example", list_vehicles, v_max=v_max, t_gap1=1)
example.initialize_variables()
example.initialize_constraints()
example.initialize_objective_function(0.99999, 0.00001)
example.MILP.optimize()
# Printing results
solution = example.getvariables(only_t=True, printing=True)
example.plot_access_times()



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
