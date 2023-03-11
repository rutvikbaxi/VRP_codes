import numpy as np
from TSP import TSP_Solver
class nCAR:

    def __init__(self, parcel_list, depot_list, C=3):
        self.parcel_list=parcel_list #list of coordinates
        self.depot_list=depot_list #list of coordinates
        self.all_node_coordinates= self.depot_list.copy() 
        self.all_node_coordinates.extend(self.parcel_list) #list of coordinates   

        self.nodes=list(range(len(self.all_node_coordinates)))  #list of nodes e.g., [0,1,2,3]
        self.remaining_nodes=self.nodes.copy()

        self.capacity=C
        self.paths=[]
        self.total_dist=0
        self.vehicles_count=0
        self.distance_matrix=self.generate_distance_matrix()

    def generate_distance_matrix(self, nodes=None):
        if nodes is None: nodes=self.nodes
        distance_matrix=[[] for _ in nodes]
        for i in range(len(nodes)):
            for j in range(len(nodes)):
                dij=self.calc_manh_dist(self.all_node_coordinates[nodes[i]], self.all_node_coordinates[nodes[j]])
                distance_matrix[i].append(dij)
        return distance_matrix

    def calc_manh_dist(self, a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    def euler_cycle(self, route):
        #returns length of a cycle
        path_length=0
        if len(route)<=1: return 0
        for i in range(len(route)-1):
            path_length+=self.distance_matrix[route[i]][route[i+1]]    
        path_length+=self.distance_matrix[route[i+1]][route[0]]
        return path_length

    def nearest_node(self, node, available_nodes):
        nearest_node=node
        nearest_distance=float("inf")
        for i in available_nodes:
            curr_dist=self.calc_manh_dist(self.all_node_coordinates[node], self.all_node_coordinates[i])
            if curr_dist <= nearest_distance:
                nearest_distance= curr_dist
                nearest_node = i

        return nearest_node

    def feasibleRoute(self):
        clusters=dict()
        best_route_length=float('inf')
        best_node=self.remaining_nodes[0]
        for node in self.remaining_nodes:
            if node==0: continue
            clusters[node]=[0]
            rem_nodes_temp=self.remaining_nodes.copy()
            for _ in range(self.capacity-1):

                if len(self.remaining_nodes)<=1: 
                    clusters[node].append(node)
                    rem_nodes_temp.remove(node)
                    break #because 

                nearest_node=self.nearest_node(node, rem_nodes_temp)
                clusters[node].append(nearest_node)
                rem_nodes_temp.remove(nearest_node)
            # rem_nodes_temp=[i for i in self.remaining_nodes if i not in clusters[node]]
            current_heuristic_length=self.euler_cycle(clusters[node]) + self.euler_cycle(rem_nodes_temp)

            if current_heuristic_length < best_route_length:
                best_route_length=current_heuristic_length
                best_node=node

        self.remaining_nodes=[i for i in self.remaining_nodes if i not in clusters[best_node]]
        # for i in range(len(self.depot_list)-1, -1, -1):
        #     self.remaining_nodes.insert(0, i)
        return clusters[best_node]

    def TSP(self, route):
        distance_matrix=self.generate_distance_matrix(route)
        best_route=TSP_Solver(route, distance_matrix)
        # best_route=route.copy()        
        return best_route, self.euler_cycle(best_route)

    def execute(self):

        while(len(self.remaining_nodes)!=0):
            new_route=self.feasibleRoute()
            shortest_path_nodes, dist = self.TSP(new_route)
            shortest_path=[self.all_node_coordinates[i] for i in shortest_path_nodes]
            self.paths.append(shortest_path)
            self.total_dist+=dist
            self.vehicles_count+=1
        return [self.paths, self.total_dist, self.vehicles_count]
    
