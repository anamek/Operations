import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import matplotlib
import numpy as np

matplotlib.use('TkAgg')

def plot_vehicle_position(vehicle_list):
    fig, ax = plt.subplots()
    max_limit = 0
    margin = 10
    # Plot the vehicle position as rectangles
    for vehicle in vehicle_list:
        if vehicle.d0 > max_limit:
            max_limit = vehicle.d0
        x = 0
        y = 0
        width = 5
        height = 10

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

def plot_access_times(ks, t_access, t_des):
    ks = np.array(ks)
    t_access = np.array(t_access)
    t_des = np.array(t_des)
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True, figsize=(8, 4))
    tN = t_access[ks == 'North']
    tNd = t_des[ks == 'North']
    for j in range(len(tN)):
        if abs(tN[j] - tNd[j]) < 0.01:
            ax1.axvline(x=tN[j], c="#FFB000", linewidth=2.5)
        else:
            ax1.axvline(x=tN[j], c="#FFB000")
            ax1.axvline(x=tNd[j], c="#FFB000", linestyle=':')
    ax1.yaxis.set_tick_params(labelleft=False)
    ax1.set_yticks([])
    ax1.set_ylabel('North')
    tS = t_access[ks == 'South']
    tSd = t_des[ks == 'South']
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
    for j in range(len(tW)):
        if abs(tW[j] - tWd[j]) < 0.01:
            ax4.axvline(x=tW[j], c="#785EF0", linewidth=2.5)
        else:
            ax4.axvline(x=tW[j], c="#785EF0")
            ax4.axvline(x=tWd[j], c="#785EF0", linestyle=':')
    ax4.yaxis.set_tick_params(labelleft=False)
    ax4.set_yticks([])
    ax4.set_xticks(range(0, int(max(max(t_access), max(t_des))+2), 2))
    ax4.set_xlim([int(min(min(t_access), min(t_des))-1), int(max(max(t_access), max(t_des))+1)])
    ax4.set_ylabel('West')
    ax4.set_xlabel('Time [s]')
    plt.show()


if __name__ == '__main__':
    pass

