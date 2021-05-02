import torch
import torch.nn as nn
import torch.optim as optim
from .layer.PointNet import *


def init_net(net, gpu_ids):
    if len(gpu_ids) > 0:
        assert(torch.cuda.is_available())
        net.cuda()
        net = torch.nn.DataParallel(net)
    return net

def define_network(opt):
    net = None
    arch = opt.arch
    name = opt.name
    gpu_ids = opt.gpu_ids

    print(arch)
    print(name)

    if name == "3DCNN":
        if arch == "C3D_Voxel_Euler":
            net == C3D_Voxel_Euler(3, 3)
        elif arch == "C3D_Voxel":
            net = C3D_Voxel_Matrix(3, 9)
    elif name == "PointNet":
        if arch == "PointNet_Pose":
            net = PointNet_Pose(3, 9)
        elif arch == "T_net_Pose":
            net = T_net_Pose(3, 9)
    elif name == "segmentation":
        if arch == "segmentation_PointNet":
            pass 
    else:
        print("Error!")

    return init_net(net, gpu_ids)


def define_loss(opt):
    if opt.dataset_mode == "pose_estimation":
        loss = nn.MSELoss()
    else:
        loss = nn.MSELoss()

    return loss


class C3D_Voxel_Euler(nn.Module):

    def __init__(self):
        super(C3D_Voxel, self, output_pos_num, output_ori_num).__init__()
        self.output_pos_num = output_pos_num
        self.output_ori_num = output_ori_num

        self.conv1 = nn.Conv3d(1, 5, kernel_size=(3, 3, 3), stride=(1, 1, 1), padding=(1, 1, 1), bias=False)
        self.conv2 = nn.Conv3d(5, 10, kernel_size=(3, 3, 3), stride=(1, 1, 1), padding=(1, 1, 1), bias=False)
        self.conv3 = nn.Conv3d(10, 5, kernel_size=(3, 3, 3), stride=(1, 1, 1), padding=(1, 1, 1), bias=False)
        self.max_pool_1 = nn.MaxPool3d(kernel_size=(3, 3, 3), stride=(2, 2, 2), padding=(1, 1, 1))
        self.max_pool_2 = nn.MaxPool3d(kernel_size=(3, 3, 3), stride=(2, 2, 2))

        self.fc_pos1 = nn.Linear(1080, 1000)
        self.fc_pos2 = nn.Linear(1000, 500)
        self.fc_pos3 = nn.Linear(500, 100)
        self.fc_pos4 = nn.Linear(100, self.output_pos_num)

        self.fc_ori1 = nn.Linear(1080, 1000)
        self.fc_ori2 = nn.Linear(1000, 500)
        self.fc_ori3 = nn.Linear(500, 100)
        self.fc_ori4 = nn.Linear(100, self.output_ori_num)

        self.relu = nn.ReLU()
        self.tan = nn.Tanh()
        self.sigmoid = nn.Sigmoid()

        self.dropout = nn.Dropout(0.5)

    def forward(self, x):
        out = self.conv1(x)
        out = self.max_pool_1(out)
        out = self.conv2(out)
        out = self.max_pool_2(out)
        out = self.conv3(out)
        out = self.max_pool_1(out)

        fl = torch.flatten(out, 1)

        h1 = self.relu(self.fc_pos1(fl))
        h1 = self.relu(self.fc_pos2(h1))
#        h1 = self.relu(self.fc_pos3(h1))
        h1 = self.dropout(h1)
        h1 = self.relu(self.fc_pos4(h1))
#        h1 = self.tan(h1)
        h1 = self.sigmoid(h1)

        h2 = self.relu(self.fc_ori1(fl))
        h2 = self.dropout(h2)
        h2 = self.relu(self.fc_ori2(h2))
        h2 = self.relu(self.fc_ori3(h2))
        h2 = self.relu(self.fc_ori4(h2))
        h2 = self.tan(h2)

        y = torch.cat([h1, h2], axis=1)

        return y


class C3D_Voxel_Matrix(nn.Module):

    def __init__(self, output_pos_num, output_ori_num):
        super(C3D_Voxel_Matrix, self).__init__()

        self.output_pos_num = output_pos_num
        self.output_ori_num = output_ori_num

        self.conv1 = nn.Conv3d(1, 5, kernel_size=3, padding=1, bias=False)
        self.conv2 = nn.Conv3d(5, 10, kernel_size=3, padding=1, bias=False)
        self.conv3 = nn.Conv3d(10, 5, kernel_size=3, padding=1, bias=False)
        self.max_pool1 = nn.MaxPool3d(kernel_size=3, stride=2, padding=1)
        self.max_pool2 = nn.MaxPool3d(kernel_size=3, stride=2)

        self.fc_pos1 = nn.Linear(1080, 1000)
        self.fc_pos2 = nn.Linear(1000, 500)
        self.fc_pos3 = nn.Linear(500, 100)
        self.fc_pos4 = nn.Linear(100, self.output_pos_num)

        self.fc_ori1 = nn.Linear(1080, 1000)
        self.fc_ori2 = nn.Linear(1000, 500)
        self.fc_ori3 = nn.Linear(500, 100)
        self.fc_ori4 = nn.Linear(100, self.output_ori_num)

        self.relu = nn.ReLU()
        self.tanh = nn.Tanh()
        self.sigmoid = nn.Sigmoid()

        self.dropout = nn.Dropout(0.5)

    def forward(self, x):
        out = self.conv1(x)
        out = self.max_pool1(out)
        out = self.conv2(out)
        out = self.max_pool2(out)
        out = self.conv3(out)
        out = self.max_pool1(out)
        fl = torch.flatten(out, 1)

        h1 = self.relu(self.fc_pos1(fl))
        h1 = self.dropout(h1)
        h1 = self.relu(self.fc_pos2(h1))
        h1 = self.relu(self.fc_pos3(h1))
        h1 = self.sigmoid(self.fc_pos4(h1))

        h2 = self.relu(self.fc_ori1(fl))
        h2 = self.dropout(h2)
        h2 = self.relu(self.fc_ori2(h2))
        h2 = self.relu(self.fc_ori3(h2))
        h2 = self.tanh(self.fc_ori4(h2))

        y = torch.cat([h1, h2], axis=1)

        return y

class PointNet_Pose(nn.Module):
    def __init__(self, out_num_pos, out_num_rot):
        super(PointNet_Pose, self).__init__()
        self.out_num_pos = out_num_pos
        self.out_num_rot = out_num_rot

        self.pointnet_vanila_global_feat = PointNet_global_feat(num_points=1024)
        self.fc_pos1 = nn.Linear(1024, 512)
        self.fc_pos2 = nn.Linear(512, 256)
        self.fc_pos3 = nn.Linear(256, 128)
        self.fc_pos = nn.Linear(128, self.out_num_pos)
        self.fc_rot1 = nn.Linear(1024, 512)
        self.fc_rot2 = nn.Linear(512, 256)
        self.fc_rot3 = nn.Linear(256, 128)
        self.fc_rot = nn.Linear(128, self.out_num_rot)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)
        self.relu = nn.ReLU()
        self.tanh = nn.Tanh()
        self.dropout = nn.Dropout(0.3)

    def forward(self, normalized_points):
        global_feat = self.pointnet_vanila_global_feat(normalized_points)
        
        h1 = self.relu(self.fc_pos1(global_feat))
        h1 = self.relu(self.fc_pos2(h1))
        h1 = self.relu(self.fc_pos3(h1))
        h1 = self.fc_pos(h1)

        h2 = self.relu(self.fc_rot1(global_feat))
        h2 = self.relu(self.fc_rot2(h2))
        h2 = self.relu(self.fc_rot3(h2))
        h2 = self.fc_rot(h2)

        y = torch.cat([h1, h2], axis=1)
        return y


class T_net_Pose(nn.Module):
    def __init__(self, out_num_pos, out_num_ori):
        super(T_net_Pose, self).__init__()
        self.out_num_pos = out_num_pos
        self.out_num_ori = out_num_ori

        self.stn_trans = STN3d_Trans(num_points=1024)
        #self.point_net = PointNet_ori(num_points=1024)

    def forward(self, global_points):
        trans_offset = self.stn_trans(global_points)
        canonical_points = global_points - trans_offset.unsqueeze(2).repeat(1, 1, global_points.size()[2])
        y = self.stn_trans(canonical_points)
        #y = self.stn_trans(global_points)
        return y

