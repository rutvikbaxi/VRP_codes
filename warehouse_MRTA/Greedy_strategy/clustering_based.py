import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import KMeans

from djikstras import Dijkstra
robot_starting_location=np.array([(0,10),(0,20)])
parcel_list=np.array([(40,30),(10,40),(20,35),(20,50),(40,40)])

class Warehourse_Router:
    '''
    Takes initial robot and parcel locations as input and performs allocation as well as routing for each
    robot.
    '''
    def __init__(self, robot_starting_location, parcel_list, ox=None, oy=None):
        self.robot_starting_location=robot_starting_location.copy()
        self.robot_list=robot_starting_location.copy()
        self.parcel_list=parcel_list.copy()
        self.mapped_parcels=dict()
        self.n_robots=len(self.robot_list)
        self.n_parcels=len(self.parcel_list)
        self.dist_travelled=[0]*self.n_robots
        self.task_complete=False
        self.robot_plot=[self.robot_list.copy()]
        self.parcel_plot=[self.parcel_list.copy()]
        self.path_planner = Dijkstra(ox, oy, 1, 1)
        self.rx=[[] for _ in range(self.n_robots)]
        self.ry=[[] for _ in range(self.n_robots)]
        self.obstacles=[ox,oy]
        self.total_distance=0
        self.execute()

    def find_nearest_parcel(self,robot_number):
        #for a specific robot find the nearest available parcel
        nearest_parcel=0
        min_dist= np.inf
        for p in range(self.n_parcels):
            l1_norm=np.linalg.norm(self.robot_list[robot_number]-self.parcel_list[p], ord=1)
            if l1_norm<min_dist: 
                nearest_parcel=p
                min_dist=l1_norm   
        return nearest_parcel,min_dist     

    def parcel_mapper(self):
        kmeans = KMeans(n_clusters=self.n_robots, random_state=0).fit(self.parcel_list)
        cluster_index = np.array(kmeans.labels_) #will be of the form [0,0,1,1,0,1] for 2 clusters
        for i in range(self.n_parcels):
            if cluster_index[i] in self.mapped_parcels:
                self.mapped_parcels[cluster_index[i]].append(self.parcel_list[i])
            else:
                self.mapped_parcels[cluster_index[i]]=[self.parcel_list[i]]
        
        for key in self.mapped_parcels:
            self.mapped_parcels[key] = sorted(self.mapped_parcels[key], key = lambda k: np.linalg.norm(k-self.robot_list[key], ord=1))

        fig=plt.figure()
        ax1=fig.add_subplot(1,1,1)
        ax1.grid(True)
        ax1.set_xlim((-10,60))
        ax1.set_ylim((-10,60))
        ax1.plot(*zip(*self.mapped_parcels[0]),'.g',markersize=15)
        ax1.plot(*zip(*self.mapped_parcels[1]),'.b',markersize=15)
        ax1.plot(self.obstacles[0], self.obstacles[1], ".k")

    def move_robot(self):
        '''
        It will allocate a parcel to each robot and move the robot to new location. If there are no more 
        parcels left, it will return all the robots to their starting locations.
        '''  
        for r in range(self.n_robots):

            if len(self.mapped_parcels[r]):
                target=self.mapped_parcels[r].pop(0)
                self.dist_travelled[r]+=np.linalg.norm(self.robot_list[r]-target, ord=1)
                self.robot_list[r]=target

            #that robot returns to home
            elif (self.robot_list[r]!=self.robot_starting_location[r]).all():
                target=self.robot_starting_location[r]
                self.dist_travelled[r]+=np.linalg.norm(self.robot_list[r]-target, ord=1)
                self.robot_list[r]=target  

            #that robot is already at home
            else:
                continue            

        self.robot_plot.append(self.robot_list.copy())
        self.parcel_plot.append(self.parcel_list.copy())

        if (self.robot_list==self.robot_starting_location).all():
            self.task_complete=True

    def plot_positions(self):
        iterations=len(self.robot_plot)
        fig=plt.figure(figsize=(3*iterations,3))
        for i in range(iterations):
            ax1=fig.add_subplot(1,iterations,i+1)
            ax1.grid(True)
            ax1.set_xlim((-10,60))
            ax1.set_ylim((-10,60))
            ax1.scatter(*zip(*self.robot_plot[i]),marker='^')
            ax1.plot(self.obstacles[0], self.obstacles[1], ".k")


            # if i<iterations-1:
            #     ax1.plot(self.rx[0][i],self.ry[0][i])
            #     ax1.plot(self.rx[1][i],self.ry[1][i])
            #     ax1.plot(self.rx[2][i],self.ry[2][i])

            ax1.plot(*zip(*self.parcel_plot[i]), ".r",markersize=5)
        plt.show()
        pass


    def execute(self):
        self.parcel_mapper()
        print(self.mapped_parcels)
        while not self.task_complete:
            self.move_robot()
        self.plot_positions()
        self.total_distance=sum(self.dist_travelled)
        print("total distance travelled: ", sum(self.dist_travelled))

ox, oy = [], []

#square boundary
for i in range(-10, 60):
    ox.append(i)
    oy.append(-10.0)
for i in range(-10, 60):
    ox.append(60.0)
    oy.append(i)
for i in range(-10, 61):
    ox.append(i)
    oy.append(60.0)
for i in range(-10, 61):
    ox.append(-10.0)
    oy.append(i)

#intermediate obstacles
for i in range(0, 50):
    ox.append(15.0)
    oy.append(i)

for i in range(0, 50):
    ox.append(35.0)
    oy.append(50.0 - i)

router=Warehourse_Router(robot_starting_location, parcel_list,ox,oy)