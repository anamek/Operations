# import matplotlib.pyplot as plt
# from MILP_OOP import Vehicle
#
# def plot_vehicle_position(vehicle_list):
#     """
#     Plot the position of a Vehicle based on its d0 attribute.
#
#     Parameters:
#     - vehicle (Vehicle): An instance of the Vehicle class.
#     """
#
#     # Assuming the road is a straight line
#     road_length = 1000  # You can adjust this based on your scenario
#
#     # Create a figure and axis
#     fig, ax = plt.subplots()
#
#     # Plot the vehicle position
#     for vehicle in vehicle_list:
#         ax.plot([vehicle.d0, vehicle.d0], [0, 1], marker='o', label=f'Vehicle {vehicle.idx}')
#
#     # Set axis labels and limits
#     ax.set_xlabel('Distance (meters)')
#     ax.set_ylabel('Position along road')
#     ax.set_xlim(0, road_length)
#     ax.set_ylim(0, 1)
#
#     # Add legend
#     ax.legend()
#
#     # Show the plot
#     plt.show()
#
# # Example usage:
# vehicle_example = Vehicle(idx=1, d0=200)
# plot_vehicle_position(vehicle_example)