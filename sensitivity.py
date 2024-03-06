from matplotlib.ticker import MaxNLocator
from MILP_OOP import MILP_Model, Vehicle
import matplotlib.pyplot as plt
import numpy as np
import matplotlib as mpl



def general_plot(x, y, x_label="x", y_label="y", title="title"):
    fig, ax = plt.subplots()
    ax.plot(x, y, linewidth=1.0)
    # ax.set(xlim=(0, 1.1*max(x)), xticks=np.arange(0, max(x), 1),
    #       ylim=(0, 1.1*max(y)), yticks=np.arange(0, 1.1*max(y),5 ))
    # Set axis labels, limits, and title
    start, end = ax.get_xlim()
    ax.xaxis.set_ticks(np.arange(2, 24, 1.0))
    ax.set_xlabel(f'{x_label}')
    ax.set_ylabel(f'{y_label}')
    ax.axhline(y=0, color='k', linewidth=1)
    plt.axhline(y=42.705, color='r', linestyle='--')
    ax.axvline(x=0, color='k', linewidth=1)
    ax.set_title(f'{title}')
    plt.show()


def default_case(vehicles=15, w1=0.5, w2=0.5, tgap1=1, tgap2=7.5, vmax=30, v_init=20, acc=3):
    list_vehicles = [Vehicle(i, v0=v_init, a_acc=acc) for i in range(vehicles)]
    example = MILP_Model(f"run", list_vehicles, t_gap1=tgap1, t_gap2=tgap2, v_max=vmax)
    example.initialize_variables()
    example.initialize_constraints()
    example.initialize_objective_function(w_1=w1, w_2=w2)
    example.optimize()
    cost = example.MILP.getObjective().getValue()
    return cost


def weight_sensitivity():
    w1_range = np.arange(0, 1.005, 0.005)
    w2_range = 1 - w1_range
    weightsrange = np.array([w1_range, w2_range]).T

    cost_list = []
    for j, row in enumerate(weightsrange):
        print(j, row)
        cost_list.append(default_case(w1=row[0], w2=row[1]))

    fig, ax = plt.subplots(layout="constrained")
    ax.plot(w1_range, cost_list, linewidth=1.0)
    plt.axhline(y=42.705, color='r', linestyle='--')

    ax.set_xlabel(f'W1 Value')
    ax.set_ylabel(f'Objective Function Cost')
    ax.set_title(f'Graph showing the Objective Cost against Weights W1 and W2', pad=20)
    ax.xaxis.set_major_locator(MaxNLocator(10))

    offset = 40
    bbox = dict(boxstyle="round", fc="0.8")
    arrowprops = dict(
        arrowstyle="->",
        connectionstyle="angle,angleA=0,angleB=90,rad=10")
    ax.annotate(
        f'Default Cost = 42.705, W1 = 0.5, W2 = 0.5',
        (w1_range[100], cost_list[100]),
        xytext=(-2 * offset, offset), textcoords='offset points',
        bbox=bbox, arrowprops=arrowprops)

    def pair(x):
        return np.around(1 - x, 1)

    def pairtf(x):
        return np.around(x, 1)

    secax = ax.secondary_xaxis('top', functions=(pairtf, pair))
    secax.set_xlabel(f'W2 Value')
    plt.show()

    return [w1_range, cost_list]


def vehicle_sensitivity():
    vehicle_range = np.arange(2, 24)
    cost_list = []

    for i in vehicle_range:
        print(i)
        cost_list.append(default_case(vehicles=i))

    general_plot(vehicle_range, cost_list,
                 x_label="Number of Vehicles",
                 y_label="Objective Function Cost",
                 title="Cost of the Objective Function against the Number of Vehicles")

    return [vehicle_range, cost_list]


# costs = vehicle_sensitivity()
# print(min(costs[1]), max(costs[1]))

def tgap1_sensitivity():
    tgap1_range = np.arange(0, 6.1, 0.1)
    tgap2_range = np.arange(0, 15.5, 0.5)
    cost_list1 = []
    cost_list2 = []

    for x in tgap1_range:
        print(x)
        cost_list1.append(default_case(tgap1=x))

    for y in tgap2_range:
        print(y)
        cost_list2.append(default_case(tgap2=y))

    fig, ax = plt.subplots()
    ax.plot(tgap1_range, cost_list1, linewidth=1.0, label="tgap 1")
    ax.plot(tgap2_range, cost_list2, linewidth=1.0, label="tgap 2")
    plt.legend()
    plt.axhline(y=42.705, color='r', linestyle='--')

    ax.set_xlabel("Time Gap (s)")
    ax.set_ylabel("Objective Cost")
    ax.set_title("Graph showing Objective Cost vs Time gap tgap 1 and tgap 2")
    plt.show()
    return [tgap1_range, cost_list1]


tgap1_sensitivity()


def tgap2_sensitivity():
    tgap1_range = np.linspace(0, 5, 10)
    tgap2_range = np.linspace(0, 15.5, 10)
    tgaprange = np.array(([tgap1_range, tgap2_range])).T
    print('shape', tgaprange.shape)
    cost_list = np.zeros(tgaprange.shape)
    print(cost_list.shape)

    X, Y = np.meshgrid(tgap1_range, tgap2_range)
    for i, t1 in enumerate(tgap1_range):
        for j, t2 in enumerate(tgap2_range):
            print(i, j)
            cost_list[i, j] = default_case(tgap1=t1, tgap2=t2)

    plt.style.use('_mpl-gallery')

    # Make data

    # Plot the surface
    fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
    ax.plot_surface(X, Y, cost_list, vmin=np.amin(cost_list), cmap=mpl.colormaps["Reds"])

    ax.set_xticks(tgap1_range)
    ax.set_yticks(tgap2_range)
    ax.set_zticks(np.linspace(np.min(cost_list), np.max(cost_list), 5))

    ax.set_xlabel('Tgap1')
    ax.set_ylabel('Tgap2')
    ax.set_zlabel('Cost')
    plt.show()
    return [tgaprange, cost_list]


def v0_and_vmax_sensitivity():
    vmax_range = np.arange(50, 60.1, 0.1)
    vmax_range = np.arange(10, 70, 0.5)

    # v0_range = np.arange(1, 50.05, 0.05)

    cost_list_vmax = []
    cost_list_v0 = []

    for x in vmax_range:
        print(x)
        cost_list_vmax.append(default_case(vmax=x))

    # for y in v0_range:
    #     print(y)
    #     cost_list_v0.append(default_case(v_init=y))

    fig, ax = plt.subplots()
    ax.plot(vmax_range, cost_list_vmax, linewidth=1.0)
    # ax.plot(v0_range, cost_list_v0, linewidth=1.0)
    plt.axhline(y=42.705, color='r', linestyle='--')

    ax.set_xlabel("Maximum Velocity Vmax [m/s]")
    ax.set_ylabel("Objective Cost")
    ax.set_title("Graph showing Objective Cost vs Maximum Velocity Vmax")


    plt.show()
    print("min cost:", min(cost_list_vmax))
    return [vmax_range, cost_list_v0]


def acc_sensitivity():
    acc_range = np.arange(0.5, 15.05, 0.05)
    cost_list = []
    for x in acc_range:
        print(x)
        cost_list.append(default_case(acc=x))

    fig, ax = plt.subplots()
    ax.plot(acc_range, cost_list, linewidth=1.0)
    plt.axhline(y=42.705, color='r', linestyle='--')

    ax.set_xlabel("Vehicle Acceleration[m/s^2]")
    ax.set_ylabel("Objective Cost")
    ax.set_title("Graph showing Objective Cost vs Maximum Vehicle Acceleration")
    plt.show()
    print(min(cost_list))

    return [acc_range, cost_list]


#
# w1_range = np.arange(0, 1.005, 0.005)
# w2_range = 1 - w1_range
# vehicle_range = np.arange(2, 15)
# tgap1_range = np.arange(0.1, 5, 1)
# tgap2_range = np.arange(1, 15.5, 0.5)
# vmax_range = np.arange(30, 105, 5)
# v0_range = np.arange(5, 55, 5)

# w1 w2 -  quick
# vehicle number [2-30] - slow after 30
# tgap1 - slow after 7
# tgap2 - slow after 20
# vmax, - constant cost
# v0, - quadratic decrease, quick
# acc - 1/x looking graph, quick
