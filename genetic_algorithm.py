import numpy as np
from scipy import spatial
import matplotlib.pyplot as plt
from sko.GA import GA_TSP


class GA:
    def __init__(self,parcel_list, depot_list, C=4, size_pop=50, max_iter=500, prob_mut=1):
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

    def calc_total_distance(self, routine):
        c=self.c
        n=len(routine)
        count=1
        path_length=self.distance_matrix[0][1]
        for i in range(1,n-1):
            if count==c:
                count=1
                path_length+=self.distance_matrix[routine[i]][0]
                path_length+=self.distance_matrix[0][routine[i+1]]

            else:
                path_length+=self.distance_matrix[routine[i]][routine[i+1]] 
                count+=1

        
        if count!=0: path_length+=self.distance_matrix[routine[i+1]][routine[0]]
        else: path_length+=2*self.distance_matrix[routine[i+1]][routine[0]]

        return path_length
        

    def points_to_paths(self, routine):
        c=self.c
        n=len(routine)
        count=0
        paths=[]
        temp=[self.depot_list[0]]
        for i in range(1,n-1):
            
            if count==c:
                count=0
                temp.append(self.depot_list[0])
                paths.append(temp)
                temp=[self.depot_list[0]]
                self.robots+=1

            else:
                temp.append(self.all_node_coordinates[i])
                count+=1
        return paths

    def execute(self):
        ga_tsp = GA_TSP(func=self.calc_total_distance, n_dim=self.n, size_pop=self.size_pop, max_iter=self.max_iter, prob_mut=self.prob_mut)
        best_points, best_distance = ga_tsp.run()
        best_points_ = np.concatenate([best_points, [best_points[0]]])
        self.all_node_coordinates= np.array(self.all_node_coordinates)
        best_points_coordinate = self.all_node_coordinates[best_points_, :]
        optimal_paths=self.points_to_paths(best_points_coordinate)
        return [optimal_paths, best_distance, self.robots, ga_tsp.generation_best_Y]




        
        


    
