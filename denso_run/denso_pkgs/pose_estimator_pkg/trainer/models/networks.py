from .discriminative import DiscriminativeLoss
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

    if name == "raugh_recognition":
        if arch == "PointNet_Pose":
            net = PointNet_Pose(3, 9)
    elif name == "object_segment":
        if arch == "JSIS3D":
            net = JSIS3D(opt.embedded_size)

    return init_net(net, gpu_ids)


def define_loss(opt):
    if opt.dataset_mode == "pose_estimation":
        loss = nn.MSELoss()
    elif opt.dataset_mode == "instance_segmentation":
        loss = DiscriminativeLoss(delta_d = opt.delta_d, delta_v = opt.delta_v)
        
    return loss


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


class PointNet_Semantic_Segmentation(nn.Module):
    def __init__(self, num_class):
        super(PointNet_Semantic_Segmentation, self).__init__()
        self.num_class=num_class
        
        self.pointnet_global_feat = PointNet_feat_segmentation(grobal_feat = False,feature_transform = True)
        self.conv1 = nn.Conv1d(1088, 512, 1)
        self.conv2 = nn.Conv1d(512, 256, 1)
        self.conv3 = nn.Conv1d(256, 128, 1)
        self.last_conv = nn.Conv1d(128, self.num_class,1)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)
        
        self.relu = nn.ReLU()
        self.soft_max=nn.LogSoftmax()
        self.dropout = nn.Dropout(0.3)

    def forward(self, x):
        batchsize = x.size()[0]
        pc_pts = x.size()[2]
        x, trans, trans_feat = self.pointnet_global_feat(x)
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.relu(self.bn2(self.conv2(x)))
        x = self.relu(self.bn3(self.conv3(x)))
        x = self.last_conv(x)
        x = x.transpose(2,1).contiguous() #memory clean for view
        x = self.soft_max(x.view(-1,self.num_class), dim=-1)
        x = x.view(batchsize, pc_pts, self.num_class)
        return x, trans_feat


class JSIS3D(nn.Module):
    def __init__(self, embedded_size):
        super(JSIS3D, self).__init__()
        self.embedded_size = embedded_size
        
        self.pointnet_global_feat = PointNet_feat_segmentation(global_feat = False,feature_transform = True)
        self.conv1 = nn.Conv1d(1088, 512, 1)
        self.conv2 = nn.Conv1d(512, 256, 1)
        self.conv3 = nn.Conv1d(256, 128, 1)
        self.last_conv = nn.Conv1d(128, self.embedded_size,1)

        self.bn1 = nn.BatchNorm1d(512)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(128)
        
        self.relu = nn.ReLU()
        self.soft_max=nn.LogSoftmax()
        self.dropout = nn.Dropout(0.3)

    def forward(self, x):
        x = self.pointnet_global_feat(x)
        x = self.relu(self.bn1(self.conv1(x)))
        x = self.relu(self.bn2(self.conv2(x)))
        x = self.relu(self.bn3(self.conv3(x)))
        x = self.last_conv(x)
        x = x.transpose(2,1).contiguous() #memory clean for view
        return x