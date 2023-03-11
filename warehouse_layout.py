import matplotlib.pyplot as plt
import numpy as np
import random

def warehouse_sample_generator():
    random.seed(a=10)
    ox,oy=[],[]
    obstacles=[ox,oy]
    #square boundary
    for i in range(0, 80):
        ox.append(i)
        oy.append(0)
    for i in range(0, 80):
        ox.append(80.0)
        oy.append(i)
    for i in range(0, 81):
        ox.append(i)
        oy.append(80.0)
    for i in range(0, 81):
        ox.append(0)
        oy.append(i)

    #intermediate obstacles

    for i in range(70, 49,-1):
        for j in range(11,80,10):
            ox.append(j)
            oy.append(i)

    for i in range(40, 19,-1):
        for j in range(11,80,10):
            ox.append(j)
            oy.append(i)

    #populating parcels
    parcel_list=[]
    for _ in range(5):
        for x in range(10,71,10):
            y=random.randint(20,40)
            if ((x,y) not in parcel_list): parcel_list.append((x,y))
            y=random.randint(50,70)
            if ((x,y) not in parcel_list): parcel_list.append((x,y))

    parcel_list=parcel_list[:2*len(parcel_list)//5]
    depot_list=[(35,43)]

    return obstacles, parcel_list, depot_list 


