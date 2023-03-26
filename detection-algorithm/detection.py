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
MODEL_UNIT_TO_INCH_COEF = 25
SAFE_NAVAGABLE_SPACE_THRESHOLD = 42

def run_detection(file):
  while True:
    # ====================================================== #
    # ==== Code below obtained from: Florent Poux, Ph.D.==== #
    # ====================================================== #
    # There are slight modifications to the original code

    # Read the point cloud data
    # data_folder="data/"
    #dataset="table.glb"
    #pcd = np.loadtxt(data_folder+dataset,skiprows=1)
    mesh = o3d.io.read_triangle_mesh(file)
    #mesh = o3d.io.read_triangle_mesh("data/"+dataset)
    pcd = mesh.sample_points_poisson_disk(number_of_points=10000, init_factor=5)
    o3d.visualization.draw_geometries([pcd])

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
          print("RETRY PROCESSING")
          continue

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

    print("HORIZONTAL SEGMENTS: ", len(horizontal_segments))
    o3d.visualization.draw_geometries(horizontal_segments + bounding_boxes)
    o3d.visualization.draw_geometries(bounding_boxes)


    print("RAMP SEGMENTS: ", sum(ramp_mask))
    # o3d.visualization.draw_geometries(np.asarray(segments_lst)[ramp_mask])

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
    bounding_box = o3d.geometry.AxisAlignedBoundingBox.create_from_points(floor_segment.points)
    bounding_box.color = [1,0,0]
    o3d.visualization.draw_geometries([floor_segment] + [bounding_box])

    space_violation_points = []
    space_violation_lines = []
    violation_ct = 0
    max_bound = bounding_box.get_max_bound()
    min_bound = bounding_box.get_min_bound()
    floor_level = max_bound[1]
    # X Direction
    for x in np.linspace(min_bound[0], max_bound[0], 20) :
      last_z = min_bound[2]
      traversal_distance = 0
      z_step = (max_bound[2] - min_bound[2])/100
      for z in np.linspace(min_bound[2], max_bound[2], 100) :
        if hull.find_simplex([x, z]) < 0 :
          traversal_distance += MODEL_UNIT_TO_INCH_COEF * z_step
        elif traversal_distance != 0:
          if traversal_distance < SAFE_NAVAGABLE_SPACE_THRESHOLD:
            print("Failure to meet safe navagable space requirements at X: %f Z: %f" % (x, z))
            print("Distance is approxiamatley %f inches" % traversal_distance)

            space_violation_lines.append([violation_ct, violation_ct+1])
            violation_ct += 2
            space_violation_points.append([x, floor_level, last_z])
            space_violation_points.append([x, floor_level, z])
          traversal_distance = 0
          last_z = z 
        else:
          traversal_distance = 0
          last_z = z 

    # Z Direction
    for z in np.linspace(min_bound[2], max_bound[2], 20) :
      last_x = min_bound[0]
      traversal_distance = 0
      x_step = (max_bound[0] - min_bound[0])/100
      for x in np.linspace(min_bound[0], max_bound[0], 100) :
        if hull.find_simplex([x, z]) < 0 :
          traversal_distance += MODEL_UNIT_TO_INCH_COEF * x_step
        elif traversal_distance != 0:
          if traversal_distance < SAFE_NAVAGABLE_SPACE_THRESHOLD:
            print("Failure to meet safe navagable space requirements at X: %f Z: %f" % (x, z))
            print("Distance is approxiamatley %f inches" % traversal_distance)

            space_violation_lines.append([violation_ct, violation_ct+1])
            violation_ct += 2
            space_violation_points.append([last_x, floor_level, z])
            space_violation_points.append([x, floor_level, z])
          traversal_distance = 0
          last_x = x
        else:
          traversal_distance = 0
          last_x = x

    colors = [[1, 0, 0] for i in range(len(space_violation_lines))]
    line_set = o3d.geometry.LineSet(
      points=o3d.utility.Vector3dVector(space_violation_points),
      lines=o3d.utility.Vector2iVector(space_violation_lines),
    )
    line_set.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([floor_segment] + [bounding_box] + [line_set])

    return len(horizontal_segments)


    





