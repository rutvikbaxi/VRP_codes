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
    data['vehicle_capacities'] = [50] * n
    data['num_vehicles'] = n
    data['demands']=[1] *len(dist_matrix)
    data['depot'] = 0
    return data

def print_solution(data, manager, routing, solution, node_list):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    time_dimension = routing.GetDimensionOrDie('Time')
    total_dist = 0
    total_load=0
    paths=[]
    route_dist=[]

    for vehicle_id in range(data['num_vehicles']):

        vehicle_path=[]
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            vehicle_path.append(node_list[manager.IndexToNode(index)])
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        vehicle_path.append(node_list[manager.IndexToNode(index)])
        plan_output+= '{}'.format(vehicle_path)
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        # print(plan_output)
        total_dist += solution.Min(time_var)

        if solution.Min(time_var)!=0: 
            paths.append(vehicle_path)
            route_dist.append(solution.Min(time_var))

    return [paths, route_dist, total_dist]

def CVRP(dist_matrix, node_list):
    """Solve the VRP with time windows."""
    # Instantiate the data problem.
    data = create_data_model(dist_matrix)

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['distance_matrix']),
                                           data['num_vehicles'], data['depot'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # # Create and register a transit callback.
    # def time_callback(from_index, to_index):
    #     """Returns the travel time between the two nodes."""
    #     # Convert from routing variable Index to time matrix NodeIndex.
    #     from_node = manager.IndexToNode(from_index)
    #     to_node = manager.IndexToNode(to_index)
    #     return data['distance_matrix'][from_node][to_node]
    
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

    # # Add Time Windows constraint.
    # time = 'Time'
    # routing.AddDimension(
    #     transit_callback_index,
    #     1,  # allow waiting time
    #     240000,  # maximum time per vehicle
    #     False,  # Don't force start cumul to zero.
    #     time)
    # time_dimension = routing.GetDimensionOrDie(time)

    # depot_idx = data['depot']
    # for vehicle_id in range(data['num_vehicles']):
    #     index = routing.Start(vehicle_id)
    #     time_dimension.CumulVar(index).SetRange(
    #         data['time_windows'][depot_idx][0],
    #         data['time_windows'][depot_idx][1])

    # # Instantiate route start and end times to produce feasible times.
    # for i in range(data['num_vehicles']):
    #     routing.AddVariableMinimizedByFinalizer(
    #         time_dimension.CumulVar(routing.Start(i)))
    #     routing.AddVariableMinimizedByFinalizer(
    #         time_dimension.CumulVar(routing.End(i)))

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
