# Article obtained from: https://towardsdatascience.com/how-to-automate-3d-point-cloud-segmentation-and-clustering-with-python-343c9039e4f5
# Author of Content: Florent Poux, Ph.D.
# Title of article: "How to automate 3D point cloud segmentation and clustering with Python"
# Code obtained from: "https://colab.research.google.com/drive/1Aygn6FxLsxYC0zOV_Rwk0CIMGOCdRw32?usp=sharing#scrollTo=g-rsI5kbbFjs"

"""
3D RANSAC implementation in python
"""
import sys
import os
import numpy as np
from sklearn import linear_model
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import open3d as o3d
from scipy.spatial import Delaunay

HORIZONTAL_Y_PROJECTION_THRESHOLD = 0.87
RAMP_Y_PROJECTION_THRESHOLD = 0.5
HORIZONTAL_POINT_THRESHOLD = 170
RAMP_POINT_THRESHOLD = 100

if __name__ == "__main__":
  # ====================================================== #
  # ==== Code below obtained from: Florent Poux, Ph.D.==== #
  # ====================================================== #
  # There are slight modifications to the original code

  # Read the point cloud data
  data_folder="data/"
  dataset="table.xyz"
  #pcd = np.loadtxt(data_folder+dataset,skiprows=1)
  pcd = o3d.io.read_point_cloud(data_folder+dataset)
  #o3d.visualization.draw_geometries([pcd])

  #pcd = pcd.voxel_down_sample(voxel_size=0.015)

  pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
  pcd.paint_uniform_color([0.6, 0.6, 0.6])

  plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
  [a, b, c, d] = plane_model
  print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
  inlier_cloud = pcd.select_by_index(inliers)
  outlier_cloud = pcd.select_by_index(inliers, invert=True)
  inlier_cloud.paint_uniform_color([1.0, 0, 0])
  outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])

  labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=10))
  max_label = labels.max()
  print(f"point cloud has {max_label + 1} clusters")

  colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
  colors[labels < 0] = 0
  pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

  segment_models={}
  segments={}
  max_plane_idx=10
  rest=pcd
  for i in range(max_plane_idx):
      colors = plt.get_cmap("tab20")(i)
      segment_models[i], inliers = rest.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
      segments[i]=rest.select_by_index(inliers)
      segments[i].paint_uniform_color(list(colors[:3]))
      rest = rest.select_by_index(inliers, invert=True)
      print("pass",i,"/",max_plane_idx,"done.")

  segment_models={}
  segments={}
  max_plane_idx=20
  rest=pcd
  d_threshold=0.01
  for i in range(max_plane_idx):
      colors = plt.get_cmap("tab20")(i)
      try:    
        segment_models[i], inliers = rest.segment_plane(distance_threshold=0.01,ransac_n=3,num_iterations=1000)
        segments[i]=rest.select_by_index(inliers)
        labels = np.array(segments[i].cluster_dbscan(eps=d_threshold*10, min_points=10))
        candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]
        best_candidate=int(np.unique(labels)[np.where(candidates==np.max(candidates))[0]])
        print("the best candidate is: ", best_candidate)
        rest = rest.select_by_index(inliers, invert=True)+segments[i].select_by_index(list(np.where(labels!=best_candidate)[0]))
        segments[i]=segments[i].select_by_index(list(np.where(labels==best_candidate)[0]))
        segments[i].paint_uniform_color(list(colors[:3]))
        print("pass",i+1,"/",max_plane_idx,"done.")
      except Exception :
        os.execv(sys.argv[0], sys.argv)

  labels = np.array(rest.cluster_dbscan(eps=0.05, min_points=5))
  max_label = labels.max()
  print(f"point cloud has {max_label + 1} clusters")

  colors = plt.get_cmap("tab10")(labels / (max_label if max_label > 0 else 1))
  colors[labels < 0] = 0
  rest.colors = o3d.utility.Vector3dVector(colors[:, :3])

  segments_lst = [segments[i] for i in range(max_plane_idx)]
  #o3d.visualization.draw_geometries(segments_lst)
  #o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])
  #o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest], zoom=0.3199,front=[0,0,0],lookat=[0,0,0],up=[0,0,1])
  #o3d.visualization.draw_geometries([rest])

  # ====================================================== #
  # ==== Code above obtained from: Florent Poux, Ph.D.==== #
  # ====================================================== #

  norms = []
  horizontal_mask = []
  ramp_mask = []
  for i in range(max_plane_idx):
    segments_lst[i].estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # Calculate z-value projections and get the average z-value projection over entire segments
    norms.append(np.mean(np.abs(np.asarray(segments_lst[i].normals)[:, 1])))
    horizontal_mask.append(norms[i]>=HORIZONTAL_Y_PROJECTION_THRESHOLD)
    ramp_mask.append(RAMP_Y_PROJECTION_THRESHOLD<=norms[i]<HORIZONTAL_Y_PROJECTION_THRESHOLD)

  #coord = o3d.geometry.TriangleMesh().create_coordinate_frame()
  #o3d.visualization.draw_geometries(segments_lst + [coord])
  o3d.visualization.draw_geometries(segments_lst)

  horizontal_segments = []
  bounding_boxes = []
  for segment in np.asarray(segments_lst)[horizontal_mask]:
     print(segment)
     if len(segment.points) > HORIZONTAL_POINT_THRESHOLD:
        print("Segment accepted")
        _, filtered_indices = segment.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.3)
        segment = segment.select_by_index(filtered_indices)
        # print(filtered_segment)
        horizontal_segments.append(segment)
        bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(segment.points)
        bounding_box.color = [1,0,0]
        bounding_boxes.append(bounding_box)

  print("Normals of each segment: ", norms)

  o3d.visualization.draw_geometries(segments_lst)

  print("HORIZONTAL SEGMENTS: ", sum(horizontal_mask))
  horizontal_segments = np.asarray(segments_lst)[horizontal_mask]
  o3d.visualization.draw_geometries(horizontal_segments)

  #print("RAMP SEGMENTS: ", sum(ramp_mask))
  #ramp_segments = np.asarray(segments_lst)[ramp_mask]
  #o3d.visualization.draw_geometries(ramp_segments)

  # All horizontal and ramp segments
  #flat_segments = np.concatenate((horizontal_segments, ramp_segments))
  #o3d.visualization.draw_geometries(flat_segments)
  flat_segments=horizontal_segments

  h_segments = []
  y_averages = []
  for s in flat_segments:
    s_arr = np.asarray(s.points)
    h_segments.append(s_arr)
    y_averages.append(np.average(s_arr[:, 1]))

  # horizontal/ramp segments ordered by descending y values 
  h_segments = [x for _, x in sorted(zip(y_averages, h_segments), reverse=True)]

  # The one segment with the lowest average z value is (probably) the floor
  floor = h_segments[len(h_segments)-1]
  # The mask used to reject floor points in the shadow of other segments
  floor_mask = []
  # All flat segments that are not the floor
  non_floor_segments = h_segments[:len(h_segments)-1]
  xz = non_floor_segments[0][:,[0,2]]
  print(xz.shape)
  for i in range(1, len(non_floor_segments)):
    np.append(xz, non_floor_segments[i][:,[0,2]], axis=0)
     
  hull = Delaunay(xz)
  for i in range(len(floor)):
    floor_mask.append(hull.find_simplex(floor[i, [0,2]])<0)

  floor = floor[floor_mask]
  
  floor_segment = o3d.geometry.PointCloud()
  floor_segment.points = o3d.utility.Vector3dVector(floor)
  o3d.visualization.draw_geometries([floor_segment])


    

  



