
import pulp
from scipy.spatial import distance

def vrp_truck(depot_list, depot_used, warehouse):
    depots_required=[depot_list[i] for i in range(len(depot_list)) if depot_used[i]==1]
    depots_required.insert(0, warehouse[0])
    n_point=len(depots_required)
    distances=distance.cdist(depots_required, depots_required, metric="cityblock")
    problem = pulp.LpProblem('tsp_mip', pulp.LpMinimize)

    # set valiables
    x = pulp.LpVariable.dicts('x', ((i, j) for i in range(n_point) for j in range(n_point)), lowBound=0, upBound=1, cat='Binary')
    # we need to keep track of the order in the tour to eliminate the possibility of subtours
    u = pulp.LpVariable.dicts('u', (i for i in range(n_point)), lowBound=1, upBound=n_point, cat='Integer')

    # set objective function
    problem += pulp.lpSum(distances[i][j] * x[i, j] for i in range(n_point) for j in range(n_point))

    # set constrains
    for i in range(n_point):
        problem += x[i, i] == 0

    for i in range(n_point):
        problem += pulp.lpSum(x[i, j] for j in range(n_point)) == 1
        problem += pulp.lpSum(x[j, i] for j in range(n_point)) == 1

    # eliminate subtour
    for i in range(n_point):
        for j in range(n_point):
            if i != j and (i != 0 and j != 0):
                problem += u[i] - u[j] <= n_point * (1 - x[i, j]) - 1
                
    # solve problem
    status = problem.solve()

    truck_route=[]
    for d in range(n_point):
        for e in range(n_point):
            if pulp.value(x[d,e])==1:
                truck_route.append((d,e))
    truck_route.sort()

    truck_cycle=[]
    start=warehouse[0][0]
    not_travelled=[1]*(n_point)

    while sum(not_travelled)!=0:
        for i in range(len(truck_route)):
            if truck_route[i][0]==start:
                truck_cycle.append(truck_route[i])
                start=truck_route[i][1]
                truck_route.remove(truck_route[i])
                not_travelled[start]=0
                break 
    truck_route_coords=[depots_required[truck_cycle[i][0]] for i in range(n_point)]
    truck_route_coords.append(depots_required[0])
    # output status, value of objective function
    return (pulp.value(problem.objective)*0.1), truck_route_coords