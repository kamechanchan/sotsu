import torch
import torch.nn as nn
import torch.nn.functional as F

class PointNet_global_feat(nn.Module):
    def __init__(self, num_points=1024):
        super(PointNet_global_feat, self).__init__()
        self.num_points = num_points

        self.conv0 = torch.nn.Conv1d(3, 32, 1)
        self.conv1 = torch.nn.Conv1d(32, 32, 1)
        self.conv2 = torch.nn.Conv1d(32, 64, 1)
        self.conv3 = torch.nn.Conv1d(64, 128, 1)
        self.conv4 = torch.nn.Conv1d(128, 256, 1)
        self.conv5 = torch.nn.Conv1d(256, 1024, 1)
        self.conv6 = torch.nn.Conv1d(1024, self.num_points, 1)
        self.max_pool = nn.MaxPool1d(self.num_points)

        self.bn0 = nn.BatchNorm1d(32)
        self.bn1 = nn.BatchNorm1d(32)
        self.bn2 = nn.BatchNorm1d(64)
        self.bn3 = nn.BatchNorm1d(128)
        self.bn4 = nn.BatchNorm1d(256)
        self.bn5 = nn.BatchNorm1d(1024)
        self.bn6 = nn.BatchNorm1d(self.num_points)

    def forward(self, x):
        x = F.relu(self.bn0(self.conv0(x)))
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))
        x = F.relu(self.bn5(self.conv5(x)))
        x = F.relu(self.bn6(self.conv6(x)))
        x = self.max_pool(x)
        x = x.view(-1, self.num_points)

        return x

class POINTNET(nn.Module):
    def __init__(self, out_num_pos, out_num_rot):
        super(POINTNET, self).__init__()
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
        self.sigmoid = nn.Sigmoid()
        self.dropout = nn.Dropout(0.3)


    def forward(self, normalized_pc):
        g_feat = self.pointnet_vanila_global_feat(normalized_pc)

        h1 = self.relu(self.fc_pos1(g_feat))
        h1 = self.relu(self.fc_pos2(h1))
        h1 = self.relu(self.fc_pos3(h1))
        pos = self.fc_pos(h1)
        # pos = self.sigmoid(self.fc_pos(h1))

        h2 = self.relu(self.fc_rot1(g_feat))
        h2 = self.relu(self.fc_rot2(h2))
        h2 = self.relu(self.fc_rot3(h2))
        rot = self.tanh(self.fc_rot(h2))

        y = torch.cat([pos, rot], axis=1)
        return y


"""
class PointNet_global_feat(nn.Module):
    def __init__(self, num_points=1024):
        super(PointNet_global_feat, self).__init__()
        self.num_points = num_points

        self.conv0 = torch.nn.Conv1d(3, 32, 1)
        self.conv1 = torch.nn.Conv1d(32, 32, 1)
        self.conv2 = torch.nn.Conv1d(32, 64, 1)
        self.conv3 = torch.nn.Conv1d(64, 128, 1)
        self.conv4 = torch.nn.Conv1d(128, 256, 1)
        self.conv5 = torch.nn.Conv1d(256, 1024, 1)
        self.conv6 = torch.nn.Conv1d(1024, self.num_points, 1)
        self.max_pool = nn.MaxPool1d(self.num_points)

        self.bn0 = nn.BatchNorm1d(32)
        self.bn1 = nn.BatchNorm1d(32)
        self.bn2 = nn.BatchNorm1d(64)
        self.bn3 = nn.BatchNorm1d(128)
        self.bn4 = nn.BatchNorm1d(256)
        self.bn5 = nn.BatchNorm1d(1024)
        self.bn6 = nn.BatchNorm1d(self.num_points)

    def forward(self, x):
        x = F.relu(self.bn0(self.conv0(x)))
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = F.relu(self.bn4(self.conv4(x)))
        x = F.relu(self.bn5(self.conv5(x)))
        x = F.relu(self.bn6(self.conv6(x)))
        x = self.max_pool(x)
        x = x.view(-1, self.num_points)

        return x

class POINTNET(nn.Module):
    def __init__(self, out_num_pos, out_num_rot):
        super(POINTNET, self).__init__()
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
        self.sigmoid = nn.Sigmoid()
        self.dropout = nn.Dropout(0.3)


    def forward(self, normalized_pc):
        g_feat = self.pointnet_vanila_global_feat(normalized_pc)

        h1 = self.relu(self.fc_pos1(g_feat))
        h1 = self.relu(self.fc_pos2(h1))
        h1 = self.relu(self.fc_pos3(h1))
        pos = self.fc_pos(h1)
        # pos = self.sigmoid(self.fc_pos(h1))

        h2 = self.relu(self.fc_rot1(g_feat))
        h2 = self.relu(self.fc_rot2(h2))
        h2 = self.relu(self.fc_rot3(h2))
        rot = self.tanh(self.fc_rot(h2))

        y = torch.cat([pos, rot], axis=1)
        return y
"""
