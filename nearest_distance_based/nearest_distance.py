import matplotlib.pyplot as plt
import numpy as np

from djikstras import Dijkstra
robot_starting_location=np.array([(0,10),(0,20),(0,30)])
parcel_list=np.array([(40,30),(10,40),(20,35),(30,10),(40,40)])

class Warehourse_Router:
    '''
    Takes initial robot and parcel locations as input and performs allocation as 
    well as routing for each robot.
    '''
    def __init__(self, robot_starting_location, parcel_list, ox=None, oy=None):
        self.robot_starting_location=robot_starting_location.copy()
        self.robot_list=robot_starting_location.copy()
        self.parcel_list=parcel_list.copy()
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
        '''
        It will allocate a parcel to each robot and move the robot to new location. If there are no more 
        parcels left, it will return all the robots to their starting locations.
        '''        
        for r in range(self.n_robots):
            if self.n_parcels==0: 
                self.task_complete=True
                break
            nearest_parcel,min_dist = self.find_nearest_parcel(r) 

            #move the robot to nearest parcel
            rx,ry=self.path_planner.planning(self.robot_list[r].copy(),self.parcel_list[nearest_parcel].copy())
            self.rx[r].append(rx)
            self.ry[r].append(ry)
            self.dist_travelled[r]+=min_dist
            self.robot_list[r]=self.parcel_list[nearest_parcel]
            self.parcel_list= np.delete(self.parcel_list,nearest_parcel, axis=0)
            self.n_parcels-=1

        if self.n_parcels==0:
            for remaining_robot in range(r,self.n_robots):
                self.rx[remaining_robot].append([self.robot_list[remaining_robot][0]])
                self.ry[remaining_robot].append([self.robot_list[remaining_robot][1]])

        self.robot_plot.append(self.robot_list.copy())
        self.parcel_plot.append(self.parcel_list.copy())

    def return_to_home(self):
        if self.task_complete:
            #return all robots to their initial positions
            for r in range(self.n_robots):
                self.dist_travelled[r]+=np.linalg.norm(self.robot_list[r]-self.robot_starting_location[r], ord=1)
                rx,ry=self.path_planner.planning(self.robot_list[r],self.robot_starting_location[r])
                self.rx[r].append(rx)
                self.ry[r].append(ry)
                self.robot_list[r]=self.robot_starting_location[r]                

        self.robot_plot.append(self.robot_list.copy())
        self.parcel_plot.append(self.parcel_list.copy())      

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

            if i<iterations-1:
                ax1.plot(self.rx[0][i],self.ry[0][i])
                ax1.plot(self.rx[1][i],self.ry[1][i])
                # ax1.plot(self.rx[2][i],self.ry[2][i])

            if len(self.parcel_plot[i]): ax1.plot(*zip(*self.parcel_plot[i]), ".r",markersize=5)
        plt.show()
        pass

    def execute(self):
        while not self.task_complete:
            self.parcel_mapper()
        self.return_to_home()
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

# router=Warehourse_Router(robot_starting_location, parcel_list,ox,oy)