import numpy as np
from scipy import spatial
import matplotlib.pyplot as plt
from sko.GA import GA_TSP
import math

class GA:
    def __init__(self, parcel_list, depot_list, C=4, size_pop=50, max_iter=500, prob_mut=1):
        self.all_node_coordinates= depot_list.copy() 
        self.depot_list=depot_list
        self.all_node_coordinates.extend(parcel_list) #list of coordinates  
        self.distance_matrix = spatial.distance.cdist(self.all_node_coordinates, self.all_node_coordinates, metric='cityblock')
        self.n=len(self.all_node_coordinates)
        self.c=C
        self.robots=0
        self.size_pop=size_pop
        self.max_iter=max_iter
        self.prob_mut=prob_mut

    def routine_arrange(self, routine):
        if np.ndim(routine)==1:
            i=np.where(routine==0)[0][0]
        else:
            i=np.where(routine==self.depot_list[0])[0][0]
        new=np.concatenate([routine[i+1:],routine[:i]])
        return new

    def calc_total_distance(self, routine):
        routine=self.routine_arrange(routine)
        c=self.c
        n=len(routine)
        depot=[0]
        self.robots=math.ceil(n/c)
        path_length=0
        for route_number in range(self.robots):
            i=route_number*c
            robot_path=np.concatenate([depot, routine[i: i+c] ,depot])
            for i in range(len(robot_path)-1):
                path_length+=self.distance_matrix[robot_path[i]][robot_path[i+1]]                

        # n=len(routine)
        # count=1
        # path_length=self.distance_matrix[0][1]
        # for i in range(1,n-1):
        #     if count==c:
        #         count=1
        #         path_length+=self.distance_matrix[routine[i]][0]
        #         path_length+=self.distance_matrix[0][routine[i+1]]

        #     else:
        #         path_length+=self.distance_matrix[routine[i]][routine[i+1]] 
        #         count+=1

        
        # if count!=0: path_length+=self.distance_matrix[routine[i+1]][routine[0]]
        # else: path_length+=2*self.distance_matrix[routine[i+1]][routine[0]]

        return path_length
        
    def points_to_paths(self, routine):
        routine=self.routine_arrange(routine)
        c=self.c
        n=len(routine)
        paths=[]
        depot=self.depot_list
        self.robots=math.ceil(n/c)

        for route_number in range(self.robots):
            i=route_number*c
            robot_path=np.concatenate([depot, routine[i: i+c] ,depot])
            paths.append(robot_path)

        return paths

    def execute(self):
        ga_tsp = GA_TSP(func=self.calc_total_distance, n_dim=self.n, size_pop=self.size_pop, max_iter=self.max_iter, prob_mut=self.prob_mut)
        best_points, best_distance = ga_tsp.run()
        # best_points_ = np.concatenate([best_points, [best_points[0]]])
        best_points_=best_points
        self.all_node_coordinates= np.array(self.all_node_coordinates)
        best_points_coordinate = self.all_node_coordinates[best_points_, :]
        optimal_paths=self.points_to_paths(best_points_coordinate)
        return [optimal_paths, best_distance, self.robots, ga_tsp.generation_best_Y]




        
        


    
