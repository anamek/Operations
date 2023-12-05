import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

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
        width = 2.5
        height = 5

        rectangle = None
        if vehicle.k == "North":
            y = -vehicle.d0
            rectangle = Rectangle((x - width / 2, y - height / 2), width, height, edgecolor='black', facecolor='blue')
        elif vehicle.k == "South":
            y = vehicle.d0
            rectangle = Rectangle((x - width / 2, y - height / 2), width, height, edgecolor='black', facecolor='blue')
        elif vehicle.k == "East":
            x = -vehicle.d0
            rectangle = Rectangle((x - height / 2, y - width / 2), height, width, edgecolor='black', facecolor='red')
        elif vehicle.k == "West":
            x = vehicle.d0
            rectangle = Rectangle((x - height / 2, y - width / 2), height, width, edgecolor='black', facecolor='red')

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

if __name__ == '__main__':
    pass

