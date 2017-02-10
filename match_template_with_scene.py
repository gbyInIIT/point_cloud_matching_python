#!/usr/bin/env python
from __future__ import print_function
import numpy as np
import libpointmatcherPythonWrapper
from tf import transformations
import math
from VTK_PointCloud import VtkPointCloud
from VTK_PointCloud_TwoView import VtkTwoViewDisplay
from VTK_PointCloud_FourView import VtkFourViewDisplay
import plyfile


def do_slc_alignment(slc_template_point_cloud_3d_npy_array, slc_scene_point_cloud_3d_npy_array):
    template_voxel_dim = 0.001
    scene_voxel_dim = 0.001
    is_force_2D = 0
    is_point_to_plane = 1
    is_icp_debug = 0
    # print("slc scene contains %d points." % slc_scene_point_cloud_3d_npy_array.shape[0])
    init_translation = np.zeros(3, dtype=np.float32)  # no translation
    template_to_scene_transformation_matrix_5x4 = libpointmatcherPythonWrapper.alignTemplateWithSceneICP(
        "what ever path:/home/gao/PycharmProjects/rosEuRocPython/slc_refined_simplified_scaled_0.01x_rotated_90x_180z.ply",
        "what ever path:/home/gao/PycharmProjects/rosEuRocPython/one_slc_on_shelf_scaled_10x.pcd",
        slc_scene_point_cloud_3d_npy_array, init_translation, slc_template_point_cloud_3d_npy_array, template_voxel_dim,
        scene_voxel_dim, is_force_2D, is_point_to_plane, is_icp_debug)
    # print("template to scene transformation matrix:")
    # print(template_to_scene_transformation_matirx)
    return template_to_scene_transformation_matrix_5x4


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


def main():
    print("Reading template ply file...", end='')
    template_ply_cloud = plyfile.PlyData.read("one_micro_switch_template.ply")
    print("Done.")
    template_point_cloud_npy_array_nx3, template_color_cloud_npy_array_nx3 = ply_obj_to_point_npy_array_nx3_and_color_nx3(template_ply_cloud)
    print("Reading scene ply file...", end='')
    scene_ply_cloud = plyfile.PlyData.read("one_micro_switch_scene.ply")
    print("Done.")
    scene_point_cloud_npy_array_nx3, scene_color_cloud_npy_array_nx3 = ply_obj_to_point_npy_array_nx3_and_color_nx3(scene_ply_cloud)

    template_alone_point_vtk_point_cloud = VtkPointCloud()
    for i, point in enumerate(template_point_cloud_npy_array_nx3):
        template_alone_point_vtk_point_cloud.addPoint(point, tuple(template_color_cloud_npy_array_nx3[i]))

    scene_alone_point_vtk_point_cloud = VtkPointCloud()
    for i, point in enumerate(scene_point_cloud_npy_array_nx3):
        scene_alone_point_vtk_point_cloud.addPoint(point, tuple(scene_color_cloud_npy_array_nx3[i]))

    init_rotation_matrix_4x4 = transformations.euler_matrix(0, 0, 0, 'rzyx')
    n_rotation = 72

    for i in range(n_rotation):
        rotation_angles = 2 * math.pi/n_rotation*i
        rotation_matrix_4x4 = transformations.euler_matrix(rotation_angles, 0, 0, 'rzyx')
        rotation_matrix_3x3 = np.dot(rotation_matrix_4x4[:3, :3], init_rotation_matrix_4x4[:3, :3])
        rotated_scene_point_cloud_npy_array_nx3 = np.dot(scene_point_cloud_npy_array_nx3, np.transpose(rotation_matrix_3x3)).astype(np.float32)
        print("ICP...")
        template_to_scene_trans_mat_5x4 = do_slc_alignment(template_point_cloud_npy_array_nx3, rotated_scene_point_cloud_npy_array_nx3)
        print("Done.")
        scene_to_template_trans_mat_4x4 = np.linalg.inv(template_to_scene_trans_mat_5x4[:4, :4])
        a, b, c = transformations.euler_from_matrix(template_to_scene_trans_mat_5x4[:4, :4], 'rzyx')
        overLap = template_to_scene_trans_mat_5x4[4, 0]
        averagedMatchingDist2 = template_to_scene_trans_mat_5x4[4, 1]
        weightedMatchingDist2 = template_to_scene_trans_mat_5x4[4, 2]
        nMatch = template_to_scene_trans_mat_5x4[4, 3]
        aligned_rotation_angles = a
        print("rotation angles: %f" % rotation_angles)
        print("rotation angles aligned: %f, overlap: %f" % (aligned_rotation_angles, overLap))
        print("rotation angles diff: %f" % np.fmod(abs(aligned_rotation_angles-rotation_angles), 2*math.pi))
        # print("%f,%f,%e,%e,%f" % (np.fmod(rotation_angles - aligned_rotation_angles, 2*math.pi),
        #                              overLap, averagedMatchingDist2, weightedMatchingDist2, nMatch), file=f)
        print("rotation angles diff:%f\noverlap:%f\naveragedMatchingDist2:%e\nweightedMatchingDist2:%e\nnMatch:%d\n" % (np.fmod(rotation_angles - aligned_rotation_angles, 2*math.pi),
                                  overLap, averagedMatchingDist2, weightedMatchingDist2, nMatch))
        template_and_scene_rotated_point_vtk_point_cloud = VtkPointCloud()
        for i, point in enumerate(rotated_scene_point_cloud_npy_array_nx3):
            template_and_scene_rotated_point_vtk_point_cloud.addPoint(point, tuple(scene_color_cloud_npy_array_nx3[i]))
        for i, point in enumerate(template_point_cloud_npy_array_nx3):
            template_and_scene_rotated_point_vtk_point_cloud.addPoint(point, tuple(template_color_cloud_npy_array_nx3[i]))

        template_and_scenen_icp_point_vtk_point_cloud = VtkPointCloud()
        scene_after_icp_point_cloud_npy_array_nx3 = np.dot(rotated_scene_point_cloud_npy_array_nx3, np.transpose(scene_to_template_trans_mat_4x4[:3, :3])) + scene_to_template_trans_mat_4x4[:3, 3]
        for i, point in enumerate(scene_after_icp_point_cloud_npy_array_nx3):
            template_and_scenen_icp_point_vtk_point_cloud.addPoint(point, tuple(scene_color_cloud_npy_array_nx3[i]))
        for i, point in enumerate(template_point_cloud_npy_array_nx3):
            template_and_scenen_icp_point_vtk_point_cloud.addPoint(point, tuple(template_color_cloud_npy_array_nx3[i]))

        # vtk_display = VtkTwoViewDisplay(slc_template_and_rotated_point_vtk_point_cloud, template_and_icp_point_vtk_point_cloud)
        vtk_display = VtkFourViewDisplay(firstPointCloud=scene_alone_point_vtk_point_cloud,
                                         secondPointCloud=template_alone_point_vtk_point_cloud,
                                         thirdPointCloud=template_and_scene_rotated_point_vtk_point_cloud,
                                         fourthPointCloud=template_and_scenen_icp_point_vtk_point_cloud)
        vtk_display.display()
    # f.close()


if __name__ == '__main__':
    main()
