from cloud_util import *
import copy
import open3d as o3d

cloud = model_loader("NHV6.pcd")

trans_init = np.asarray([[0.862, 0.011, -0.507, 100],
                         [-0.139, 0.967, -0.215, 10],
                         [0.487, 0.255, 0.835, -10], 
                         [0.0, 0.0, 0.0, 1.0]])

source_cloud = copy.deepcopy(cloud).scale(0.001, center=False)
target_cloud = copy.deepcopy(cloud)
mesh = o3d.geometry.create_mesh_coordinate_frame(0.01)
mesh_tx = copy.deepcopy(mesh).translate((1.3,0,0))


source_cloud.paint_uniform_color([1, 0, 0])
target_cloud.paint_uniform_color([0, 0, 1])

target_cloud = target_cloud.transform(trans_init)

# target_cloud.scale(0.01)


# pcd_viewer([mesh, source_cloud])
pcd_viewer([source_cloud])


# trans = p2pICP(source_cloud, target_cloud, trans_init)


target_cloud = source_cloud.transform(trans)
pcd_viewer([source_cloud, target_cloud])
# draw_registration_result(source_cloud, target_cloud, trans)

# pcd_viewer([cloud, target_cloud])

