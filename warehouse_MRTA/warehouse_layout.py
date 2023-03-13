import matplotlib.pyplot as plt
import numpy as np
import random

def warehouse_sample_generator(a=10, n_customers=20):
    random.seed(a=a)
    ox,oy=[],[]
    obstacles=[ox,oy]
    width=35
    breadth=110
    #square boundary
    for i in range(0, breadth):
        ox.append(i)
        oy.append(0)
    for i in range(0, width):
        ox.append(breadth)
        oy.append(i)
    for i in range(0, breadth+1):
        ox.append(i)
        oy.append(width)
    for i in range(0, width+1):
        ox.append(0)
        oy.append(i)

    #intermediate obstacles

    for y in range(30, 19,-1):
        for x in range(10,110,10):
            ox.append(x)
            oy.append(y)

    for y in range(15, 4,-1):
        for x in range(10,110,10):
            ox.append(x)
            oy.append(y)

    #populating parcels
    parcel_list=[]
    n_customers_added=0
    while n_customers_added<n_customers:        
        for x in range(11,111,10):
            y=random.randint(5,15)
            while ((x,y) in parcel_list): y=random.randint(5,15)
            parcel_list.append((x,y))
            n_customers_added+=1

            y=random.randint(20,30)
            while ((x,y) in parcel_list): y=random.randint(20,30)
            parcel_list.append((x,y))
            n_customers_added+=1

            if n_customers>100:
                x=x-2
                y=random.randint(5,15)
                while ((x,y) in parcel_list): y=random.randint(5,15)
                parcel_list.append((x,y))
                n_customers_added+=1

                y=random.randint(20,30)
                while ((x,y) in parcel_list): y=random.randint(20,30)
                parcel_list.append((x,y))
                n_customers_added+=1


    depot_list=[(55,17)]

    return obstacles, parcel_list, depot_list 


