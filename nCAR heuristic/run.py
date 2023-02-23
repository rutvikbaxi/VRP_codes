from warehouse_layout import warehouse_sample_generator
from warehouse_plotter import plot_warehouse
from nCAR import nCAR

obstacles, parcel_list, depot_list = warehouse_sample_generator()

#using nCAR
parcel_mapper_nCAR= nCAR(parcel_list=parcel_list, depot_list=depot_list, C=3)
[paths, dist, n_robots]=parcel_mapper_nCAR.execute()
plot_warehouse(obstacles[0], obstacles[1], parcel_list, robot_paths=paths, depot_list=depot_list)
print(dist)
print(n_robots)


