import matplotlib.pyplot as plt
import numpy as np
def plotter(ox,oy,parcel_list, depot_list=None, warehouse=None, robot_paths=None, truck_route=None):
    colours=['r','b','k', 'c', 'm', 'y']
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(1, 1, 1)
    city_size=max(max(parcel_list))
    # Major ticks every 20, minor ticks every 5
    # major_ticks = np.arange(0, 101, 20)
    # minor_ticks = np.arange(0, 101, 1)
    # ax.set_xticks(major_ticks)
    # ax.set_xticks(minor_ticks, minor=True)
    # ax.set_yticks(major_ticks)
    # ax.set_yticks(minor_ticks, minor=True)
    # ax.grid(which='minor', alpha=0.2)
    # ax.grid(which='major', alpha=0.5)

    #obstacles
    ax.scatter(ox,oy,s=5, marker='s', alpha=0.5)

    #customers
    x, y = zip(*parcel_list)
    ax.plot(x, y, "ro", markersize=5)

    #depots
    if depot_list:
        x, y = zip(*depot_list)
        ax.plot(x, y, "k^", markersize=10)

    #warehouse
    x, y = zip(*warehouse)
    ax.plot(x, y, "ms", markersize=13)

    #truck_route
    if truck_route:
        x, y = zip(*truck_route)
        ax.plot(x, y, "g", lw=4)

    #robot_paths
    if robot_paths:
        for i in range(len(robot_paths)):
            if robot_paths[i][0]==(0,0): depot_number=i
            else: depot_number=depot_list.index(robot_paths[i][0]) #depot_list.index(robot_paths[i][0])
            x, y = zip(*robot_paths[i])
            ax.plot(x,y, colours[depot_number%5])

    # Major ticks every 20, minor ticks every 5
    major_ticks = np.arange(0, city_size, 1)
    # minor_ticks = np.arange(0, 101, 5)

    ax.set_xticks(major_ticks)
    ax.set_xticklabels( ax.get_xticks(),rotation=90)
    # ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    # ax.set_yticks(minor_ticks, minor=True)

    # And a corresponding grid
    ax.grid(which='both', alpha=0.3)
    plt.show()