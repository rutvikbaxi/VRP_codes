from city_layout import warehouse_sample_generator
from scipy.spatial import distance
from layout_plotter import plotter
from models.CVRP import CVRP
from pulp import *
import pandas as pd

class TWO_EVRP:
    def __init__(self, city_size=5, n_customers=45, n_depots=7, time_limit=180, n_robots_per_depot=3) -> None:
        self.obstacles, self.customer_list, self.depot_list = warehouse_sample_generator(city_size=city_size, n_customers=n_customers, n_depots=n_depots)
        self.DEPOTS=range(0,len(self.depot_list))
        self.ROBOTS=range(0,n_robots_per_depot) #max robots at eah depot
        self.CUSTOMERS=range(0,n_customers)
        self.v_robot= 3  #kmph 
        self.block_size=0.1 #km
        self.t_max_robot=8 #hours, max working time of robot in a day
        self.tf = 0.75 #hours taken for full charge
        self.battery_life = 3 #max operating time on one charge (hours)
        self.service_time = 1/15 #time to service a customer (h)
        self.distance_matrix=distance.cdist(self.depot_list, self.customer_list, metric="cityblock")*self.block_size #km
        self.warehouse=[(0,0)]
        self.tdc = 2*self.distance_matrix/self.v_robot #round trip time, hours
        self.combinations=[(r,d) for r in self.ROBOTS for d in self.DEPOTS]
        self.time=0
        self.time_limit=time_limit
        self.truck_velocity=30 #kmph
        self.max_distance_robot=int(self.v_robot*self.battery_life/2)
        
        self.depot_customer_mapping=dict()
        for d in self.DEPOTS:
            self.depot_customer_mapping[d]=[customer for customer in self.CUSTOMERS if self.distance_matrix[d][customer]<=self.max_distance_robot]
        self.customer_optimized_set=[(r,c,d) for r in self.ROBOTS for c in self.CUSTOMERS for d in self.DEPOTS if c in self.depot_customer_mapping[d]]
        self.customer_optimized_set.sort()

    def optimal_depots(self):

        x = LpVariable.dicts("Choice", self.customer_optimized_set, cat="Binary")#X_rcd
        o = LpVariable.dicts("hub_existence", (self.DEPOTS), cat="Binary") #o_d
        model = pulp.LpProblem("Optimal_Hub_Count_Model", LpMinimize)
        model+=pulp.lpSum(o[i] for i in self.DEPOTS)
        
        #constraints
        for c in self.CUSTOMERS:
            model+= (
                lpSum(x[(r,c,d)] for (r,d) in self.combinations if (r,c,d) in set(self.customer_optimized_set)) ==1,
                f"Customer {c} is assigned to at least one hub"
            )

        for r in self.ROBOTS:
            for d in self.DEPOTS:
                model+= (
                    lpSum((self.tdc[d][c]*(1+self.tf/self.battery_life) + self.service_time)*x[(r,c,d)] for c in self.CUSTOMERS if (r,c,d) in set(self.customer_optimized_set))<=self.t_max_robot,
                    f"Maximum working time of robot {r} at depot {d} in a day"
                )       
        for c in self.CUSTOMERS:
            for d in self.DEPOTS:
                model+= (
                    lpSum(x[(r,c,d)] for r in self.ROBOTS if (r,c,d) in set(self.customer_optimized_set))<=o[d],
                    f"Only one robot should be there between depot {d} and customer {c}"
                ) 
        # model.writeLP("models/optimal_p_value.lp")
        model.solve(GUROBI(msg=True, timeLimit=self.time_limit))
        print("Status:", LpStatus[model.status])
        self.p=value(model.objective)
        self.time+=model.solutionTime

    def p_median(self):
  
        # x = LpVariable.dicts("Choice", (self.ROBOTS, self.CUSTOMERS, self.DEPOTS), cat="Binary")#X_rcd
        x = LpVariable.dicts("Choice", self.customer_optimized_set, cat="Binary")#X_rcd

        o = LpVariable.dicts("hub_existence", (self.DEPOTS), cat="Binary") #o_d
        model = pulp.LpProblem("Optimal_Hub_Count_Model", LpMinimize)
        model+=pulp.lpSum(self.tdc[d][c]*x[(r,c,d)] for r in self.ROBOTS for c in self.CUSTOMERS for d in self.DEPOTS if (r,c,d) in set(self.customer_optimized_set))

        #constraints
        for c in self.CUSTOMERS:
            model+= (
                lpSum(x[(r,c,d)] for (r,d) in self.combinations if (r,c,d) in set(self.customer_optimized_set)) ==1,
                f"Customer {c} is assigned to at least one hub"
            )
        for r in self.ROBOTS:
            for d in self.DEPOTS:
                model+= (
                    lpSum((self.tdc[d][c]*(1+self.tf/self.battery_life) + self.service_time)*x[(r,c,d)] for c in self.CUSTOMERS if (r,c,d) in set(self.customer_optimized_set))<=self.t_max_robot,
                    f"Maximum working time of robot {r} at depot {d} in a day"
                )       
        for c in self.CUSTOMERS:
            for d in self.DEPOTS:
                model+= (
                    lpSum(x[(r,c,d)] for r in self.ROBOTS if (r,c,d) in set(self.customer_optimized_set))<=o[d],
                    f"Only one robot should be there between depot {d} and customer {c}"
                ) 
        model+= (
            lpSum(o[d] for d in self.DEPOTS)==self.p,
            "total p medians should be selected"
        )

        # model.writeLP("models/p_depot_location.lp")
        model.solve(GUROBI(msg=True, timeLimit=self.time_limit))
        print("Status:", LpStatus[model.status])
        
        self.robot_paths=[]
        for c in self.CUSTOMERS:
            for d in self.DEPOTS:
                if sum([value(x[(r,c,d)]) for r in self.ROBOTS if (r,c,d) in set(self.customer_optimized_set)]):
                    self.robot_paths.append([self.depot_list[d], self.customer_list[c]])
        
        self.depot_used=[int(value(o[d])) for d in self.DEPOTS]
        self.robot_total_dist=value(model.objective) * self.v_robot
        self.time+=model.solutionTime
        self.robot_arcs_variable=x
        self.n_robots_used=len(set((d,r) for (r,c,d) in self.customer_optimized_set if int(value(self.robot_arcs_variable[r,c,d]))==1))
    
    def vrp_truck(self):
        depots_required=[self.depot_list[i] for i in range(len(self.depot_list)) if self.depot_used[i]==1]
        depots_required.insert(0, self.warehouse[0])
        n_point=len(depots_required)
        distances=distance.cdist(depots_required, depots_required, metric="cityblock") *self.block_size
        problem = pulp.LpProblem('tsp_mip', LpMinimize)

        x = pulp.LpVariable.dicts('x', ((i, j) for i in range(n_point) for j in range(n_point)), lowBound=0, upBound=1, cat='Binary')
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
                    
        problem.solve(GUROBI(msg=True, timeLimit=self.time_limit))
        print("Status VRP:", LpStatus[problem.status])

        truck_route=[]
        for d in range(n_point):
            for e in range(n_point):
                if pulp.value(x[d,e])==1:
                    truck_route.append((d,e))
        truck_route.sort()

        truck_cycle=[]
        start=self.warehouse[0][0]
        not_travelled=[1]*(n_point)

        while sum(not_travelled)!=0:
            for i in range(len(truck_route)):
                if truck_route[i][0]==start:
                    truck_cycle.append(truck_route[i])
                    start=truck_route[i][1]
                    truck_route.remove(truck_route[i])
                    not_travelled[start]=0
                    break 
        self.truck_route_coords=[depots_required[truck_cycle[i][0]] for i in range(n_point)]
        self.truck_route_coords.append(depots_required[0])
        self.time+=problem.solutionTime

        self.truck_distance=pulp.value(problem.objective)     

    def cvrp_truck_only(self):
        self.all_nodes_truck=self.customer_list.copy()
        self.all_nodes_truck.insert(0, self.warehouse[0])
        self.truck_dist_matrix= distance.cdist(self.all_nodes_truck, self.all_nodes_truck, metric="cityblock")*100 #in metres
        
        arr=[[] for _ in range(len(self.truck_dist_matrix[0]))]
        for r in range(len(self.truck_dist_matrix)):
            for c in range(len(self.truck_dist_matrix[0])):
                arr[r].append(int(self.truck_dist_matrix[r][c]))
        solution=CVRP(arr, self.all_nodes_truck)
        self.truck_only_routes=solution[0]
        
        plotter(self.obstacles[0], self.obstacles[1], self.customer_list, self.depot_list, robot_paths=self.truck_only_routes, warehouse=self.warehouse)
        
        self.truck_distance_only=solution[1]/1000
        

    def calculate_cost(self):
        self.charging_cost=5*0.25*0.75 # 5Rs/kWh * 0.25kW * 0.75h
        truck_mileage=20
        petrol_cost=100
        # n_times_charging_required=0
        # for d in self.DEPOTS:
        #     for r in self.ROBOTS:
        #         distance_per_robot=sum(self.distance_matrix[d][c]*value(self.robot_arcs_variable[(r,c,d)]) for c in self.CUSTOMERS if (r,c,d) in self.customer_optimized_set)
        #         n_times_charging_required+=int(distance_per_robot/self.max_distance_robot)
        # print(n_times_charging_required)

        self.robot_cost_charging=(self.robot_total_dist/self.max_distance_robot) * self.charging_cost

        self.truck_cost=self.truck_distance/ truck_mileage *petrol_cost #km/ (km/l) * Rs/l 
        self.truck_cost+= len(self.CUSTOMERS)//50 * 

    def execute(self):
        # self.optimal_depots()
        # self.p_median()
        # self.vrp_truck()
        self.cvrp_truck_only()
        # plotter(self.obstacles[0], self.obstacles[1], self.customer_list, self.depot_list, robot_paths=self.robot_paths, warehouse=self.warehouse, truck_route=self.truck_route_coords)
        self.calculate_cost()

customers=200
city_size=6
solver=TWO_EVRP(city_size, customers, n_depots=6, time_limit=300, n_robots_per_depot=5)
solver.execute()
df=pd.DataFrame(columns=['#customers', "#depots", "#total_robot_distance", "#computational_time", "city_size"])
df.loc[len(df)]=[customers, solver.p, solver.robot_total_dist, solver.time, city_size]

print(df)
