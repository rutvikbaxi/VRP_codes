from warehouse_layout import warehouse_sample_generator
from warehouse_plotter import plot_warehouse
from nCAR_heuristic.nCAR import nCAR
from genetic_algorithm import GA
import matplotlib.pyplot as plt
obstacles, parcel_list, depot_list = warehouse_sample_generator()

#using nCAR
# parcel_mapper_nCAR= nCAR(parcel_list=parcel_list, depot_list=depot_list, C=4)
# [paths, dist, n_robots]=parcel_mapper_nCAR.execute()

#usingGA
parcel_mapper=GA(parcel_list, depot_list, C=4, size_pop=50, max_iter=300, prob_mut=0.05)
[paths, dist, n_robots, progress]=parcel_mapper.execute()
plt.plot(progress)
plt.show()
plot_warehouse(obstacles[0], obstacles[1], parcel_list, robot_paths=paths, depot_list=depot_list)
print(dist)
print(n_robots)


