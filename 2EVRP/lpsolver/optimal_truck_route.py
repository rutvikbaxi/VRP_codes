from pulp import *
from scipy.spatial import distance

def truck_router(parcel_list, depot_list, p):
    ROBOTS=range(0,3) #max robots at eah depot
    CUSTOMERS=range(0,len(parcel_list))

    warehouse=(0,0)
    depot_list.insert(0, warehouse)

    DEPOTS=range(len(depot_list))
    velocity_robot= 3  #kmph 
    block_size=0.1 #km
    M=8 #hours, max working time of robot in a day
    tf = 1 #hours taken for full charge
    b = 3 #max operating time on one charge (hours)
    ts = 1/15 #time to service a customer (h)
    distance_matrix=distance.cdist(depot_list, parcel_list, metric="cityblock")*block_size
    
    distance_matrix_depots=distance.cdist(depot_list, depot_list, metric="cityblock")*block_size

    # max_dist_reachable=int(velocity_robot*b/2)
    # reachable_customers=[]
    # for d in DEPOTS:
    #     feasible_Set=[customer for customer in CUSTOMERS if distance_matrix[d][customer]<=max_dist_reachable]
    #     reachable_customers.append(feasible_Set)

    combinations=[(r,d) for r in ROBOTS for d in DEPOTS[1:]]
    tdc = 2*distance_matrix/velocity_robot #round trip time
    n=len(DEPOTS)
    q_quantity=[1]*n
    x = LpVariable.dicts("Choice", (ROBOTS, CUSTOMERS, DEPOTS[1:]), cat="Binary", lowBound=0, upBound=1)#X_rcd
    o = LpVariable.dicts("hub_existence", (DEPOTS[1:]), cat="Binary", lowBound=0, upBound=1) #o_d
    y= LpVariable.dicts("route", (DEPOTS, DEPOTS), cat="Binary", lowBound=0, upBound=1)
    u= LpVariable.dicts("dummy", (DEPOTS[1:]), lowBound=0)
    z=LpVariable.dicts("capacity", (DEPOTS), cat="Binary", lowBound=0, upBound=1)

    model = pulp.LpProblem("Optimal_Hub_Count_Model", LpMinimize)
    model+= lpSum(distance_matrix_depots[d][e]* y[d][e] for d in DEPOTS for e in DEPOTS if d!=e)

    for i in DEPOTS:
        model+= y[i][i]==0

    model+= lpSum(y[d][0] for d in DEPOTS[1:])==1
    model+= lpSum(y[0][d] for d in DEPOTS[1:])==1

    for e in DEPOTS[1:]:
        model+= lpSum(y[d][e] for d in DEPOTS[1:])==lpSum(y[e][d] for d in DEPOTS[1:])

    for e in DEPOTS[1:]:
        model+= lpSum(y[d][e] for d in DEPOTS[1:])<=1

    for d in DEPOTS[1:]: 
        model+= u[d]>=1
        model+= u[d]<=n

    for d in DEPOTS[1:]:
        for e in DEPOTS[1:]:
            if d!=e:  model+= u[d]-u[e] + 1 <= (n)*(1-y[d][e])

    model+=lpSum(z[d] for d in DEPOTS[1:])==p

    for d in DEPOTS[1:]:
        model+= q_quantity[d] >= z[d]
        model+= z[d]<=o[d]
        model+=lpSum(y[d][e] for e in DEPOTS[1:])==o[d]

    


     
    

    # model+=lpSum(o[d] for d in DEPOTS[1:])==p
    # for e in DEPOTS[1:]:
    #     model+= lpSum(y[d][e] for d in DEPOTS[1:])==o[e]
    # for d in range(2,p+1):
    #     for e in range(2, p+1):
    #         if d==e: continue
    #         else: model+= u[d]-u[e] + 1 <= (p)*(1-y[d][e])
    # for d in DEPOTS[1:]:
    #     model+= u[d] <= (p+1) *o[d] 

    #original_constraints
    for c in CUSTOMERS:
        model+= (
            lpSum(x[r][c][d] for (r,d) in combinations) ==1,
            f"Customer {c} is assigned to at least one hub"
        )

    for r in ROBOTS:
        for d in DEPOTS[1:]:
            model+= (
                lpSum((tdc[d][c]*(1+tf/b) + ts)*x[r][c][d] for c in CUSTOMERS)<=M,
                f"Maximum working time of robot {r} at depot {d} in a day"
            )       

    for c in CUSTOMERS:
        for d in DEPOTS[1:]:
            model+= (
                lpSum(x[r][c][d] for r in ROBOTS)<=o[d],
                f"Only one robot should be there between depot {d} and customer {c}"
            ) 

    model.writeLP("truck_router.lp")
    model.solve(GUROBI(msg=True, timeLimit=180))
    print("Status:", LpStatus[model.status])
    arcs=[]
    truck_route=[]

    for c in CUSTOMERS:
        for d in DEPOTS[1:]:
            if sum([value(x[r][c][d]) for r in ROBOTS]):
                arcs.append([depot_list[d], parcel_list[c]])


    truck_route=[]
    for d in range(n):
        for e in range(n):
            if pulp.value(y[d][e])==1:
                truck_route.append((d,e))
    truck_route.sort()

    n=len(truck_route)
    print(truck_route)
    truck_cycle=[]
    start=(0,0)
    not_travelled=[1]*(n)

    while sum(not_travelled)!=0:
        print(not_travelled)
        for i in range(len(truck_route)):
            if truck_route[i][0]==start:
                truck_cycle.append(truck_route[i])
                start=truck_route[i][1]
                truck_route.remove(truck_route[i])
                not_travelled[start]=0
                break 
    truck_route_coords=[depot_list[truck_cycle[i][0]] for i in range(n)]
    truck_route_coords.append(depot_list[0])
    return model, arcs, truck_route

