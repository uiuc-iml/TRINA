from vision import PointCloud
import pcl
import cv2
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def numpy_to_pcd(input_array):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(input_array)
    return pcd

def list_to_pcd(input_list):

    """
    input: n*3 list
    """

    array = np.array(input_list)
    return numpy_to_pcd(array)

# def extract_plane(PointCloud, Ksearch = 50, dis_threshold = 0.01, distance_Weight = 0.01, max_iter = 100):
#
#     """
#     input: PointCloud:open3d pointcloud
#     Perform plane extraction with ransac
#     Return
#     planes -- a pointcloud of the plane
#     coefficients -- a list of plane parameters
#     """
#
#     array = np.asarray(PointCloud.points, dtype = np.float32)
#     list = array.tolist()
#     cloud  = pcl.PointCloud()
#     cloud.from_array(array)
#     planes = []
#     seg = cloud.make_segmenter_normals(ksearch=Ksearch)
#     seg.set_optimize_coefficients(True)
#     seg.set_model_type(pcl.SACMODEL_NORMAL_PLANE)
#     seg.set_method_type(pcl.SAC_RANSAC)
#     seg.set_distance_threshold(dis_threshold)
#     seg.set_normal_distance_weight(distance_Weight)
#     seg.set_max_iterations(max_iter)
#     plane_indices, coefficients = seg.segment()
#
#     if len(plane_indices) == 0:
#         print('Could not estimate a planar model for the given pointcloud')
#         return None
#
#
#     for j, indices in enumerate(plane_indices):
#
#         points = list[indices]
#         planes.append(points)
#
#     planes = list_to_pcd(planes)
#
#     return (planes, coefficients)

def extract_plane(PointCloud, distance_threshold = 0.01, ransac_n = 3, num_iterations = 1000):
    plane_model, inliers = PointCloud.segment_plane(distance_threshold, ransac_n, num_iterations)
    return (plane_model, PointCloud.select_down_sample(inliers))

def remove_plane(PointCloud, distance_threshold = 0.01, ransac_n = 3, num_iterations = 1000):
    plane_model, inliers = PointCloud.segment_plane(distance_threshold, ransac_n, num_iterations)
    return (plane_model, PointCloud.select_down_sample(inliers, invert = True)) # sometimes here may be select_by_index, depend on which version is being used.

def DBSCAN_clustering(PointCloud):

    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))
    labels = np.array(labels)
    max_label = labels.max()
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    PointCloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
    return PointCloud, labels

def segmented_object(PointCloud):
    labels = DBSCAN_clustering(PointCloud)[1]

    points = np.asarray(PointCloud.points)
    length = labels.max() + 1
    list = []
    for i in range(length):
        list.append([])
    if len(labels) != len(points):
        raise Exception("the length of the labels and points are different")
    for i in range(len(labels)):
        if labels[i] == -1:
            continue
        list[labels[i]].append(points[i].tolist())
    ptc_list = []
    for ptc in list:
        ptc_list.append(list_to_pcd(ptc))

    return ptc_list

def vision_segment(PointCloud, dis_thres = 0.02, MinClusterSize = 10, MaxClusterSize = 25000):

    array = np.asarray(PointCloud.points, dtype = np.float32)
    cloud  = pcl.PointCloud()
    cloud.from_array(array)

    segments = []
    tree = cloud.make_kdtree()
    ec = cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(dis_thres)
    ec.set_MinClusterSize(MinClusterSize)
    ec.set_MaxClusterSize(MaxClusterSize)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    print('cluster_indices : ' + str(cluster_indices.count) + " count.")

    for j, indices in enumerate(cluster_indices):

        points = array[indices]
        segment = numpy_to_pcd(points)
        segments.append(segment)

    print(len(segments))

    return segments

def remove_floor(PointCloud, z = 0.3):
    list = np.asarray(PointCloud.points).tolist()
    new_list = []
    for element in list:
        if element[2] > z:
            new_list.append(element)

    return list_to_pcd(new_list)

def remove_wall(PointCloud, x = 5.0):
    list = np.asarray(PointCloud.points).tolist()
    new_list = []
    for element in list:
        if element[0] < x:
            new_list.append(element)

    return list_to_pcd(new_list)

def remove_stuff_below_table(PointCloud, plane):
    list = np.asarray(PointCloud.points).tolist()
    new_list = []
    for element in list:
        if plane[0]* element[0] +  plane[1]* element[1] +  plane[2]* element[2] +  plane[3] > 0:
            new_list.append(element)

    return list_to_pcd(new_list)

def remove_noise(PointCloud, k = 50, std_mul = 1):

    """
    Remove noises in the pointcloud using pcl's statistical outlier removal

    k -- Number of points (k) to use for mean distance estimation int
    std_mul -- Standard deviation multiplier threshold float
    """

    array = np.asarray(PointCloud.points, dtype = np.float32)
    cloud  = pcl.PointCloud()
    cloud.from_array(array)

    fil = cloud.make_statistical_outlier_filter()
    fil.set_mean_k (k)
    fil.set_std_dev_mul_thresh (std_mul)
    filtered = fil.filter()

    return numpy_to_pcd(np.array(filtered.to_array()))



if __name__ == '__main__':
    scene = o3d.io.read_point_cloud('pcd_samples/sample_table.pcd')
    o3d.visualization.draw_geometries([scene])
    scene = remove_floor(scene)
    o3d.visualization.draw_geometries([scene])
    scene = remove_wall(scene)
    o3d.visualization.draw_geometries([scene])
    plane, scene = remove_plane(scene)
    o3d.visualization.draw_geometries([scene])
    scene = remove_stuff_below_table(scene, plane)
    o3d.visualization.draw_geometries([scene])

    list_object = vision_segment(scene)
    for object in list_object:
        i = 0
        o3d.visualization.draw_geometries([object])
        o3d.io.write_point_cloud("Segmented_Object_" + i.str() + ".pcd", pcd)
        i+=1
    # scene = remove_plane(scene)
    # scene = remove_noise(scene)
    # o3d.visualization.draw_geometries([scene])
    # scene = remove_plane(scene)
    # scene = remove_noise(scene)
    # o3d.visualization.draw_geometries([scene])

    # [a,b,c,d], scene = extract_plane(scene)
    # o3d.visualization.draw_geometries([scene])
    # print([a,b,c,d])
    # scene = remove_plane(scene)
    # labeled_scene, labels = DBSCAN_clustering(scene)
    # one_object = segmented_object(scene)
    #
    # for i in range(len(one_object)):
    #     o3d.visualization.draw_geometries([one_object[i]])

    # points = np.array(scene.points)
    # pcd = PointCloud(points)
    # pcd.removeNoise()

    # segments = pcd.segment_regionGrowing()
    #
    # for i, segment in enumerate(segments):
    #
    #     segment.save_to_ply(str(i) + '.ply')
