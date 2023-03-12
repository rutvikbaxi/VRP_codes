from warehouse_layout import warehouse_sample_generator
from warehouse_plotter import plot_warehouse
from nCAR_heuristic.nCAR import nCAR
from genetic_algorithm import GA
import matplotlib.pyplot as plt
from icecream import install
install()
import pandas as pd
import time

df=pd.DataFrame(columns=["#parcels", "#capacity_robot", "#GA_distance", "nCAR_dist", "t_GA", "t_nCAR"])
n_parcels=[100,200]
capacity =[ 3,  4]

for p in n_parcels:
    for c in capacity:
        df_temp=pd.DataFrame(columns=["#parcels", "#capacity_robot", "#GA_distance", "nCAR_dist",  "t_GA", "t_nCAR"])
        for _ in range(3):
            obstacles, parcel_list, depot_list = warehouse_sample_generator(a=10, n_customers=p) 
            t_on=time.time()
            parcel_mapper= nCAR(parcel_list=parcel_list, depot_list=depot_list, C=5)
            [paths, dist_heuristic, n_robots, progress]=parcel_mapper.execute()
            t_nCAR=time.time()-t_on

            t_on=time.time()
            parcel_mapper=GA(parcel_list, depot_list, C=5, size_pop=50, max_iter=300, prob_mut=0.05)
            [paths, dist_GA, n_robots, progress]=parcel_mapper.execute()
            t_GA=time.time()-t_on

            df_temp.loc[len(df_temp.index)]=[p, c, dist_GA, dist_heuristic, t_GA, t_nCAR]
        df.loc[len(df.index)]=df_temp.mean()
df.to_csv("results.csv", index=False)

# plot_warehouse(obstacles[0], obstacles[1], parcel_list, robot_paths=paths, depot_list=depot_list)

ic(df)


