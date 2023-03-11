from city_layout import warehouse_sample_generator
from layout_plotter import plotter

from lpsolver.optimal_depot_ip import optimal_depots
from lpsolver.dual_objective import dual_objective
from lpsolver.p_median import p_median
from lpsolver.vrp_truck import vrp_truck
from lpsolver.optimal_truck_route import truck_router

from pulp import *

warehouse=[(0,0)]
[obstacles, parcel_list, depot_list] = warehouse_sample_generator(city_size=5, n_customers=45)

# model, arcs =optimal_depots(parcel_list, depot_list)
# p=value(model.objective)
# model_p_median, depot_used, arcs = p_median(parcel_list, depot_list, p)

# model, arcs = dual_objective(parcel_list, depot_list)
truck_route=None
# model_vrp_truck, truck_route = vrp_truck(depot_list, depot_used, warehouse)
model, arcs, truck_route=truck_router(parcel_list, depot_list, p=3)
plotter(obstacles[0], obstacles[1], parcel_list, depot_list, robot_paths=None, warehouse=warehouse, truck_route=truck_route)

print()