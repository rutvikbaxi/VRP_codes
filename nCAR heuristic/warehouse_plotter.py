import matplotlib.pyplot as plt
import numpy as np
def plot_warehouse(ox,oy,parcel_list, robot_paths, depot_list):
    colours=['r','g','b','k', 'c', 'm', 'y']
    fig = plt.figure(figsize=(10,10))
    ax = fig.add_subplot(1, 1, 1)

    # Major ticks every 20, minor ticks every 5
    major_ticks = np.arange(0, 101, 20)
    minor_ticks = np.arange(0, 101, 1)
    ax.set_xticks(major_ticks)
    ax.set_xticks(minor_ticks, minor=True)
    ax.set_yticks(major_ticks)
    ax.set_yticks(minor_ticks, minor=True)
    ax.grid(which='minor', alpha=0.2)
    ax.grid(which='major', alpha=0.5)

    #obstacles
    ax.scatter(ox,oy,s=5, marker='s', alpha=0.5)

    #parcels
    x, y = zip(*parcel_list)
    ax.plot(x, y, ".r", markersize=5)

    #depots
    ax.plot(depot_list[0][0], depot_list[0][1],".g", markersize=10)

    #robot_paths
    for i in range(len(robot_paths)):
        x, y = zip(*robot_paths[i])
        ax.plot(x,y,colours[i])
    plt.show()