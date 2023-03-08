#!/usr/bin/env python3.6
import numpy as np
import rospy
import math
from ctypes import *
import struct
from scipy import stats
import sensor_msgs.point_cloud2 as pc2
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from collections import Counter
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
import open3d 
import matplotlib.pyplot as plt
FIELDS_XYZ = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]
FIELDS_XYZRGB = FIELDS_XYZ + \
    [PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)]
BIT_MOVE_16 = 2**16
BIT_MOVE_8 = 2**8
convert_rgbUint32_to_tuple = lambda rgb_uint32: (
    (rgb_uint32 & 0x00ff0000)>>16, (rgb_uint32 & 0x0000ff00)>>8, (rgb_uint32 & 0x000000ff)
)
convert_rgbFloat_to_tuple = lambda rgb_float: convert_rgbUint32_to_tuple(
    int(cast(pointer(c_float(rgb_float)), POINTER(c_uint32)).contents.value)
)
def callback(ros_cloud):
     # Get cloud data from ros_cloud
    field_names=[field.name for field in ros_cloud.fields]
    cloud_data = list(pc2.read_points(ros_cloud, skip_nans=True, field_names = field_names))

    # Check empty
    open3d_cloud = open3d.geometry.PointCloud()
    if len(cloud_data)==0:
        print("Converting an empty cloud")
        return None

    # Set open3d_cloud
    IDX_RGB_IN_FIELD=3 # x, y, z, rgb
        
    # Get xyz
    xyz = [(x,y,z) for x,y,z,rgb in cloud_data ] # (why cannot put this line below rgb?)

    # Get rgb
    # Check whether int or float
    if type(cloud_data[0][IDX_RGB_IN_FIELD])==float: # if float (from pcl::toROSMsg)
        rgb = [convert_rgbFloat_to_tuple(rgb) for x,y,z,rgb in cloud_data ]
    else:
        rgb = [convert_rgbUint32_to_tuple(rgb) for x,y,z,rgb in cloud_data ]

    # combine
    open3d_cloud.points = open3d.utility.Vector3dVector(np.array(xyz))
    open3d_cloud.colors = open3d.utility.Vector3dVector(np.array(rgb)/255.0)
    #Rotate to get proper orientation
    open3d_cloud = open3d_cloud.rotate(open3d_cloud.get_rotation_matrix_from_zyx((np.pi,np.pi,0)))
    #open3d_cloud1 = open3d_cloud.rotate(open3d_cloud.get_rotation_matrix_from_xyz((0.0392,-0.0265, -0.0609)))
    #open3d_cloud1 = open3d_cloud.translate((-0.1575,-0.1875,0.2797))
    #Visualize
    
    #pen3d.visualization.draw_geometries([open3d_cloud])
    #open3d.io.write_point_cloud("Copy.pcd", open3d_cloud)
    #open3d.visualization.draw_geometries([open3d_cloud1])
    #print("hi")
    
    #out_of_plane = plane_segmentation(open3d_cloud) #Plane Segmentation
    #open3d.visualization.draw_geometries([out_of_plane])
    #find_clusters(out_of_plane) #Clustering of Point Clouds
    

def plane_segmentation(cloud_point):
    plane_model, inliers = cloud_point.segment_plane(distance_threshold=0.01, ransac_n = 10, num_iterations=1000)
   
    [a,b,c,d] = plane_model
    inlier_cloud = cloud_point.select_by_index(inliers)
    outlier_cloud = cloud_point.select_by_index(inliers, invert = True)
    return outlier_cloud 

def find_clusters(cloud_point):
    max = 0
    with open3d.utility.VerbosityContextManager(open3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(cloud_point.cluster_dbscan(eps=0.01, min_points = 10, print_progress=True))
    labels = labels[labels != -1]
    high_label = stats.mode(labels)[0]
    print(high_label)
    cluster_cloud = cloud_point.select_by_index((np.where(labels==high_label)[0]))
    open3d.visualization.draw_geometries([cluster_cloud])
    print(np.mean(cluster_cloud.points,axis=0)) 
    ''' max_label = labels.max()
    print(stats.mode(labels[labels!=-1]))
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    cloud_point.colors = open3d.utility.Vector3dVector(colors[:, :3])'''
    open3d.visualization.draw_geometries([cluster_cloud])

def find_distances(cloud_point):
    points = np.array(cloud_point.points)
    center = np.array([0,0,0])
    radius = 1.5

    distances = np.linalg.norm(points-center, axis=1)
    #cloud_point.points = open3d.utility.Vector3dVector(points[distances<=radius])


def pointCloudListener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, callback, queue_size=1, buff_size=52428800)
    rospy.spin()


if __name__ == '__main__':
    pointCloudListener()