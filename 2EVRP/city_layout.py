import matplotlib.pyplot as plt
import numpy as np
import random

def warehouse_sample_generator(city_size=2, n_customers=15, n_depots=5,random_seed=10):
    random.seed(a=random_seed)
    ox,oy=[],[]
    obstacles=[ox,oy]
    #square boundary
    n=city_size*10
    for i in range(0, n):
        ox.append(i)
        oy.append(0)
    for i in range(0, n):
        ox.append(n)
        oy.append(i)
    for i in range(0, n+1):
        ox.append(i)
        oy.append(n)
    for i in range(0, n+1):
        ox.append(0)
        oy.append(i)

    customer_list=[]
    n_customers_added=0
    while n_customers_added<n_customers:
        x=random.randint(0,n)
        y=random.randint(0,n) 
        if ((x,y) not in customer_list): 
             customer_list.append((x,y))
             n_customers_added+=1

    depot_list=[]
    counter=0
    while counter<n_depots:
        x=random.randint(2,n-2)
        y=random.randint(2,n-2) 
        if ((x,y) not in depot_list): 
             depot_list.append((x,y))
             counter+=1
             
    return obstacles, customer_list, depot_list


