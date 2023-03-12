"""Vehicles Routing Problem (VRP) with Time Windows."""

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp


def create_data_model(dist_matrix):
    """Stores the data for the problem."""
    data = {}
    data['distance_matrix'] = dist_matrix
    data['time_windows'] = [
        (0, 5),  # depot
        (7, 12),  # 1
        (10, 15),  # 2
        (16, 18),  # 3
        (10, 13),  # 4
        (0, 5),  # 5
        (5, 10),  # 6
        (0, 4),  # 7
        (5, 10),  # 8
        (0, 3),  # 9
        (10, 16),  # 10
        (10, 15),  # 11
        (0, 5),  # 12
        (5, 10),  # 13
        (7, 8),  # 14
        (10, 15),  # 15
        (11, 15),  # 16
    ]
    n=(4 + len(dist_matrix)//100)
    data['vehicle_capacities'] = [55] * n
    data['num_vehicles'] = n
    data['demands']=[1] *len(dist_matrix)
    data['depot'] = 0
    return data
def print_solution(data, manager, routing, solution, node_list):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    total_distance = 0
    total_load = 0
    paths=[]
    truckwise_dist=[]

    for vehicle_id in range(data['num_vehicles']):
        vehicle_path=[]
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        route_distance = 0
        route_load = 0
        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            vehicle_path.append(node_list[manager.IndexToNode(node_index)])
            route_load += data['demands'][node_index]
            plan_output += ' {0}-> '.format(node_index)
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(
                previous_index, index, vehicle_id)
        # plan_output += ' {0} Load({1})\n'.format(manager.IndexToNode(index),
        #                                          route_load)
        plan_output += 'Distance of the route: {}m\n'.format(route_distance)
        # plan_output += 'Load of the route: {}\n'.format(route_load)
        # print(plan_output)
        total_distance += route_distance
        total_load += route_load
        if route_distance:
            paths.append(vehicle_path)
            truckwise_dist.append(route_distance)
    print('Total distance of all routes: {}m'.format(total_distance))
    return paths, total_distance, truckwise_dist

def CVRP(dist_matrix, node_list):
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model(dist_matrix)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def distance_callback(from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data['distance_matrix'][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Capacity constraint.
    def demand_callback(from_index):
        """Returns the demand of the node."""
        # Convert from routing variable Index to demands NodeIndex.
        from_node = manager.IndexToNode(from_index)
        return data['demands'][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(
        demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,  # null capacity slack
        data['vehicle_capacities'],  # vehicle maximum capacities
        True,  # start cumul to zero
        'Capacity')
    
    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
        k=print_solution(data, manager, routing, solution, node_list)
        return k
    
# from city_layout import warehouse_sample_generator
# from scipy.spatial import distance
# import numpy as np
# _,cust,_=warehouse_sample_generator(city_size=2, n_customers=5, n_depots=4)
# t=distance.cdist(cust, cust, metric="cityblock")

# arr=[[] for _ in range(len(t[0]))]
# for r in range(len(t)):
#     for c in range(len(t[0])):
#         arr[r].append(int(t[r][c]))
# print(arr)
# CVRP(arr)
