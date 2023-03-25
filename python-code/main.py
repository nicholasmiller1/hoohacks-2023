import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
import plotly.express as px

# load xyz file data and then read it into a 2d table (N x 6) of the XYZRGB points of 3d model
data_folder = "data/"
dataset="desk.xyz"
pcd = np.loadtxt(data_folder+dataset,skiprows=1)

xyz=pcd[:,:3]
rgb=pcd[:,3:6]

plt.figure(figsize=(8, 5), dpi=150)
plt.scatter(xyz[:,0], xyz[:,1], c=rgb/255, s=0.05)
plt.title("Top-View")
plt.xlabel('X-axis (m)')
plt.ylabel('Y-axis (m)')
plt.show()


fig = px.scatter(x=xyz[:,0], y=xyz[:,1], color=xyz[:,2], threshold=0.05, iterations=1000)
fig.show()

# =========== AUTOMATIC PARAMETER INITIALIZATION CODE ========= #


#from sklearn.neighbors import KDTree
#tree = KDTree(np.array(xyz), leaf_size=2)  
#tree.query(xyz, k=8)