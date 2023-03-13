from warehouse_layout import warehouse_sample_generator
from warehouse_plotter import plot_warehouse
from nCAR_heuristic.nCAR import nCAR
from genetic_algorithm import GA
import matplotlib.pyplot as plt
from icecream import install
install()
import pandas as pd
import time
from tqdm import tqdm

df=pd.DataFrame(columns=["#parcels", "#capacity_robot", "#GA_distance", "nCAR_dist", "t_GA", "t_nCAR"])
n_parcels=[200, 100, 60]
capacity =[ 3, 4]

n_parcels=[40]
capacity=[5]

for p in tqdm(n_parcels, leave=True):
    for c in tqdm(capacity, leave=False, desc=f"parcels {p}"):
        df_temp=pd.DataFrame(columns=["#parcels", "#capacity_robot", "#GA_distance", "nCAR_dist",  "t_GA", "t_nCAR"])
        for instance in tqdm(range(3), desc=f"capacity {c}", leave=False):
            obstacles, parcel_list, depot_list = warehouse_sample_generator(a=instance, n_customers=p) 

            t_on=time.time()
            parcel_mapper= nCAR(parcel_list=parcel_list, depot_list=depot_list, C=c)
            [paths, dist_heuristic, n_robots, progress]=parcel_mapper.execute()
            t_nCAR=time.time()-t_on

            t_on=time.time()
            parcel_mapper=GA(parcel_list, depot_list, C=c, size_pop=50, max_iter=300, prob_mut=0.05)
            [paths1, dist_GA, n_robots, progress]=parcel_mapper.execute()
            t_GA=time.time()-t_on

            df_temp.loc[len(df_temp.index)]=[p, c, dist_GA, dist_heuristic, t_GA, t_nCAR]
        df.loc[len(df.index)]=df_temp.mean()

df.to_csv("results.csv", index=False)

plot_warehouse(obstacles[0], obstacles[1], parcel_list, robot_paths=paths, depot_list=depot_list)

ic(df)


