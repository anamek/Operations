import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib
import numpy as np
from matplotlib.lines import Line2D

matplotlib.use('TkAgg')

def plot_vehicle_position(vehicle_list):
    fig, ax = plt.subplots()
    max_limit = 0
    margin = 10
    max_limit = max(vehicle.d0 for vehicle in vehicle_list) * 1.05
    # Plot the vehicle position as rectangles
    for vehicle in vehicle_list:
        x = 0
        y = 0
        width = max_limit/60
        height = max_limit/30

        rectangle = None
        if vehicle.k == "North":
            y = -vehicle.d0
            rectangle = Rectangle((x - width / 2, y - height / 2), width, height, edgecolor='blue', facecolor='blue')
        elif vehicle.k == "South":
            y = vehicle.d0
            rectangle = Rectangle((x - width / 2, y - height / 2), width, height, edgecolor='blue', facecolor='blue')
        elif vehicle.k == "East":
            x = -vehicle.d0
            rectangle = Rectangle((x - height / 2, y - width / 2), height, width, edgecolor='red', facecolor='red')
        elif vehicle.k == "West":
            x = vehicle.d0
            rectangle = Rectangle((x - height / 2, y - width / 2), height, width, edgecolor='red', facecolor='red')

        ax.add_patch(rectangle)

        # Uncomment this if you want to see the vehicles labelled
        #ax.text(x, y, f'Vehicle {vehicle.idx}', ha='left', va='top', color='black')

    # Set axis labels, limits, and title
    ax.set_xlabel('X Distance (meters)')
    ax.set_ylabel('Y Distance (meters)')

    ax.set_xlim(- max_limit - margin, max_limit + margin)
    ax.set_ylim(- max_limit - margin, max_limit + margin)
    ax.set_box_aspect(1)
    ax.axhline(y=0, color='k', linewidth=1)
    ax.axvline(x=0, color='k', linewidth=1)
    ax.set_title('Vehicle Positions')

    # Adding Legend
    north_south_patch = Rectangle((0, 0), 1, 1, edgecolor='black', facecolor='blue', label='North/South Bound')
    east_west_patch = Rectangle((0, 0), 1, 1, edgecolor='black', facecolor='red', label='East/West Bound')
    ax.legend(handles=[north_south_patch, east_west_patch], fontsize="small")

    # Show the plot
    plt.show()

def plot_access_times(ks, t_access, t_des, signals=False):
    ks = np.array(ks)
    t_access = np.array(t_access)
    t_des = np.array(t_des)
    if signals:
        fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, sharex=True, figsize=(8, 5), height_ratios=[3, 3, 3, 3, 1])
    else:
        fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True, figsize=(8, 4))
    tN = t_access[ks == 'North']
    tNd = t_des[ks == 'North']
    if len(tN) > 1:
        aN, sizeN = sorted(tN), len(tN)
        resN = [aN[i + 1] - aN[i] for i in range(sizeN) if i + 1 < sizeN]
        minN = min(resN)
    else:
        minN = float('inf')
    for j in range(len(tN)):
        if abs(tN[j] - tNd[j]) < 0.01:
            ax1.axvline(x=tN[j], c="#FFB000", linewidth=2.5)
        else:
            ax1.axvline(x=tN[j], c="#FFB000")
            ax1.axvline(x=tNd[j], c="#FFB000", linestyle=':')
    ax1.yaxis.set_tick_params(labelleft=False)
    ax1.set_yticks([])
    ax1.set_ylabel('North')
    custom_lines = [Line2D([0], [0], lw=1.5),
                    Line2D([0], [0], ls=':'),
                    Line2D([0], [0], lw=2.5)]
    ax1.legend(custom_lines, ['Access Time', 'Desired Time', 'Both'], fontsize="7")
    tS = t_access[ks == 'South']
    tSd = t_des[ks == 'South']
    if len(tS) > 1:
        aS, sizeS = sorted(tS), len(tS)
        resS = [aS[i + 1] - aS[i] for i in range(sizeS) if i + 1 < sizeS]
        minS = min(resS)
    else:
        minS = float('inf')
    for j in range(len(tS)):
        if abs(tS[j] - tSd[j]) < 0.01:
            ax2.axvline(x=tS[j], c="#FE6100", linewidth=2.5)
        else:
            ax2.axvline(x=tS[j], c="#FE6100")
            ax2.axvline(x=tSd[j], c="#FE6100", linestyle=':')
    ax2.yaxis.set_tick_params(labelleft=False)
    ax2.set_yticks([])
    ax2.set_ylabel('South')
    tE = t_access[ks == 'East']
    tEd = t_des[ks == 'East']
    if len(tE) > 1:
        aE, sizeE = sorted(tE), len(tE)
        resE = [aE[i + 1] - aE[i] for i in range(sizeE) if i + 1 < sizeE]
        minE = min(resE)
    else:
        minE = float('inf')
    for j in range(len(tE)):
        if abs(tE[j] - tEd[j]) < 0.01:
            ax3.axvline(x=tE[j], c="#DC267F", linewidth=2.5)
        else:
            ax3.axvline(x=tE[j], c="#DC267F")
            ax3.axvline(x=tEd[j], c="#DC267F", linestyle=':')
    ax3.yaxis.set_tick_params(labelleft=False)
    ax3.set_yticks([])
    ax3.set_ylabel('East')
    tW = t_access[ks == 'West']
    tWd = t_des[ks == 'West']
    if len(tW) > 1:
        aW, sizeW = sorted(tW), len(tW)
        resW = [aW[i + 1] - aW[i] for i in range(sizeW) if i + 1 < sizeW]
        minW = min(resW)
    else:
        minW = float('inf')
    for j in range(len(tW)):
        if abs(tW[j] - tWd[j]) < 0.01:
            ax4.axvline(x=tW[j], c="#785EF0", linewidth=2.5)
        else:
            ax4.axvline(x=tW[j], c="#785EF0")
            ax4.axvline(x=tWd[j], c="#785EF0", linestyle=':')
    ax4.yaxis.set_tick_params(labelleft=False)
    ax4.set_yticks([])
    ax4.set_xticks(range(0, int(max(max(t_access), max(t_des))+3), 2))
    # ax4.set_xlim([int(min(min(t_access), min(t_des))-1), int(max(max(t_access), max(t_des))+1)])
    ax4.set_xlim([0, int(max(max(t_access), max(t_des)) + 2)])
    ax4.set_ylabel('West')
    ax4.set_xlabel('Time [s]')
    if signals:
        k_sorted = [x for _, x in sorted(zip(t_access, ks))]
        k_sorted = list(map(lambda x: x.replace('North', 'Vertical'), k_sorted))
        k_sorted = list(map(lambda x: x.replace('South', 'Vertical'), k_sorted))
        k_sorted = list(map(lambda x: x.replace('East', 'Horizontal'), k_sorted))
        k_sorted = list(map(lambda x: x.replace('West', 'Horizontal'), k_sorted))
        t_sorted = sorted(t_access)
        for i in range(len(t_sorted)):
            if k_sorted[i] == 'Vertical':
                ax5.add_patch(Rectangle((t_sorted[i], 0), 0.01, 1, color="blue"))
            else:
                ax5.add_patch(Rectangle((t_sorted[i], 0), 0.01, 1, color="purple"))
            if (i < len(t_sorted)-1) and (k_sorted[i] == k_sorted[i+1]):
                if k_sorted[i] == 'Vertical':
                    ax5.add_patch(Rectangle((t_sorted[i], 0), t_sorted[i+1]-t_sorted[i], 1, color="blue"))
                else:
                    ax5.add_patch(Rectangle((t_sorted[i], 0), t_sorted[i + 1] - t_sorted[i], 1, color="purple"))
            elif i < len(t_sorted)-1:
                assert(t_sorted[i + 1] - t_sorted[i] > 7.4999)
        ax5.yaxis.set_tick_params(labelleft=False)
        ax5.set_yticks([])
    assert(min(minW, minN, minS, minE) > 0.999)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    pass

