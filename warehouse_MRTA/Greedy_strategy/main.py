from nearest_distance import Warehourse_Router as dist_based_router
from clustering_based import Warehourse_Router as kmeans_based_router
import numpy as np

robot_starting_location=np.array([(0,10),(0,20)])
parcel_list=np.array([(10,40),(20,35),(20,50),(50,40),(40,20)])

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

router1=dist_based_router(robot_starting_location, parcel_list,ox,oy)
print("1st router over")
router2=kmeans_based_router(robot_starting_location, parcel_list,ox,oy)
print(router1.total_distance, router2.total_distance)