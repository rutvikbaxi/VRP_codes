import numpy as np
a=[[3,4],[3,1],[6,8],[3,0],[4,6]]
# print(sorted(a, key = lambda k: np.linalg.norm(a[k]-[3,0], ord=1)))
def dist(x):
    return np.linalg.norm(x-np.array([3,0]), ord=1)
print(sorted(a, key = lambda k: np.linalg.norm(k-np.array([3,0]), ord=1)))
