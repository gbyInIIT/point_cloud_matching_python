#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import math
from VTK_PointCloud import VtkPointCloud
from VTK_PointCloud_TwoView import VtkTwoViewDisplay
from VTK_PointCloud_FourView import VtkFourViewDisplay
import plyfile


def ply_obj_to_point_npy_array_nx3_and_color_nx3(ply_obj):
    x_npy_array_nx1 = np.array(ply_obj['vertex']['x'])
    y_npy_array_nx1 = np.array(ply_obj['vertex']['y'])
    z_npy_array_nx1 = np.array(ply_obj['vertex']['z'])
    r_npy_array_nx1 = np.array(ply_obj['vertex']['red'])
    g_npy_array_nx1 = np.array(ply_obj['vertex']['green'])
    b_npy_array_nx1 = np.array(ply_obj['vertex']['blue'])
    cloud_npy_array_nx6 = np.transpose(np.vstack((x_npy_array_nx1, y_npy_array_nx1, z_npy_array_nx1, r_npy_array_nx1, g_npy_array_nx1, b_npy_array_nx1)))
    point_cloud_npy_array_nx3 = cloud_npy_array_nx6[:, :3]
    point_cloud_npy_array_nx3 -= np.mean(point_cloud_npy_array_nx3, axis=0)
    return point_cloud_npy_array_nx3, cloud_npy_array_nx6[:, 3:6]


def update_ply_obj(ply_obj, new_point_cloud_npy_array_nx3):
    ply_obj['vertex']['x'] = new_point_cloud_npy_array_nx3[:, 0]
    ply_obj['vertex']['y'] = new_point_cloud_npy_array_nx3[:, 1]
    ply_obj['vertex']['z'] = new_point_cloud_npy_array_nx3[:, 2]
    pass


def main():
    print("Reading template ply file...", end='')
    template_ply_cloud = plyfile.PlyData.read("/home/gao/Downloads/ElasticFusion/cmake-build-release/bin/collimator_in_cern_clean.ply")
    print("Done.")
    template_point_cloud_npy_array_nx3, template_color_cloud_npy_array_nx3 = ply_obj_to_point_npy_array_nx3_and_color_nx3(template_ply_cloud)
    template_point_cloud_npy_array_nx3 -= np.mean(template_point_cloud_npy_array_nx3, axis=0)
    update_ply_obj(template_ply_cloud, template_point_cloud_npy_array_nx3)
    plyfile.PlyData()
    template_ply_cloud.text = False
    template_ply_cloud.byte_order = '<'
    template_ply_cloud.write("/home/gao/Downloads/ElasticFusion/cmake-build-release/bin/collimator_in_cern_clean_centered.ply")
    # print("Reading scene ply file...", end='')
    # scene_ply_cloud = plyfile.PlyData.read("one_micro_switch_scene.ply")
    # print("Done.")
    # scene_point_cloud_npy_array_nx3, scene_color_cloud_npy_array_nx3 = ply_obj_to_point_npy_array_nx3_and_color_nx3(scene_ply_cloud)


if __name__ == '__main__':
    main()
