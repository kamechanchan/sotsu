import torch
import torch.nn as nn
import torch.optim as optim
import os
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/'))
sys.path.append(os.path.join(os.path.dirname(__file__), '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/arch/'))


from .arch.C3D_VOXEL import C3D_VOXEL
from .arch.POINTNET import POINTNET

def init_net(net, gpu_ids):
    if len(gpu_ids) > 0:
        assert(torch.cuda.is_available())
        net.cuda()
        net = torch.nn.DataParallel(net)
    return net

def define_network(opt):
    net = None
    arch = opt.arch
    gpu_ids = opt.gpu_ids

    if arch == "3DCNN":
        
        net = C3D_VOXEL(3, 3)
    elif arch == "PointNet_Pose":
        
        net = POINTNET(3, 9)
    else:
        print("networks.py: Error!! arch is incorrect.Please check arch")
    return init_net(net, gpu_ids)


def define_loss(opt):
    if opt.dataset_mode == "pose_estimation":
        loss = nn.MSELoss()
    else:
        loss = nn.MSELoss()

    return loss


