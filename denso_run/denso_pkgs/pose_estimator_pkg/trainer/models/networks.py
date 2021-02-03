import torch
import torch.nn as nn
import torch.optim as optim


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
        from arch.C3D_VOXEL import*
        net = C3D_VOXEL(3, 9)
    elif arch == "PointNet_Pose":
        from arch.POINTNET import POINTNET
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


