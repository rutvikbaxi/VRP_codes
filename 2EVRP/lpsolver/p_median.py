from pulp import *
from scipy.spatial import distance

def p_median(parcel_list, depot_list, p):
    DEPOTS=range(0,len(depot_list))
    ROBOTS=range(0,3) #max robots at eah depot
    CUSTOMERS=range(0,len(parcel_list))
    city_size=2 #km
    velocity_robot= 3  #kmph 
    block_size=0.1 #km
    M=8 #hours, max working time of robot in a day
    tf = 1 #hours taken for full charge
    b = 3 #max operating time on one charge (hours)
    ts = 1/15 #time to service a customer (h)
    distance_matrix=distance.cdist(depot_list, parcel_list, metric="cityblock")*block_size

    # max_dist_reachable=int(velocity_robot*b/2)
    # reachable_customers=[]
    # for d in DEPOTS:
    #     feasible_Set=[customer for customer in CUSTOMERS if distance_matrix[d][customer]<=max_dist_reachable]
    #     reachable_customers.append(feasible_Set)

    combinations=[(r,d) for r in ROBOTS for d in DEPOTS]
    tdc = 2*distance_matrix/velocity_robot #round trip time

    prob=LpProblem("Distribution_Problem")

    x = LpVariable.dicts("Choice", (ROBOTS, CUSTOMERS, DEPOTS), cat="Binary")#X_rcd
    o = LpVariable.dicts("hub_existence", (DEPOTS), cat="Binary") #o_d

    model = pulp.LpProblem("Optimal_Hub_Count_Model", LpMinimize)
    model+=pulp.lpSum(tdc[d][c]*x[r][c][d] for r in ROBOTS for c in CUSTOMERS for d in DEPOTS)

    # for w in Warehouses:
    #     prob += (
    #         lpSum([vars[w][b] for b in Bars]) <= supply[w],
    #         f"Sum_of_Products_out_of_Warehouse_{w}",
    #     )

    #constraints
    for c in CUSTOMERS:
        model+= (
            lpSum(x[r][c][d] for (r,d) in combinations) ==1,
            f"Customer {c} is assigned to at least one hub"
        )

    for r in ROBOTS:
        for d in DEPOTS:
            model+= (
                lpSum((tdc[d][c]*(1+tf/b) + ts)*x[r][c][d] for c in CUSTOMERS)<=M,
                f"Maximum working time of robot {r} at depot {d} in a day"
            )       

    for c in CUSTOMERS:
        for d in DEPOTS:
            model+= (
                lpSum(x[r][c][d] for r in ROBOTS)<=o[d],
                f"Only one robot should be there between depot {d} and customer {c}"
            ) 

    model+= (
        lpSum(o[d] for d in DEPOTS)==p,
        "total pe medians should be selected"
    )

    model.writeLP("p_depot_location.lp")
    model.solve()
    print("Status:", LpStatus[model.status])
    arcs=[]

    for c in CUSTOMERS:
        for d in DEPOTS:
            if sum([value(x[r][c][d]) for r in ROBOTS]):
                arcs.append([depot_list[d], parcel_list[c]])
    
    print(value(model.objective), "using #p_median minimization")
    depots=[int(value(o[d])) for d in DEPOTS]
    return model, depots, arcs

