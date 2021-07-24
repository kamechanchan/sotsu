from __future__ import print_function
import torch
import torch.nn as nn
import torch.nn.parallel
import torch.utils.data
from torch.autograd import Variable
import numpy as np
import torch.nn.functional as F
import utils


class STN3d_Trans(nn.Module):
    def __init__(self, num_points=1024):
        super(STN3d_Trans, self).__init__()

        self.num_points = num_points

        self.conv1 = torch.nn.Conv1d(3, 128, 1)
        self.conv2 = torch.nn.Conv1d(128, 256, 1)
        self.conv3 = torch.nn.Conv1d(256, self.num_points, 1)
        self.max_pool = nn.MaxPool1d(self.num_points)
        self.fc1 = nn.Linear(self.num_points, 256)
        self.fc2 = nn.Linear(256, 128)
        self.fc3 = nn.Linear(128, 3)
        self.relu = nn.ReLU()

    def forward(self, x):
        x = F.relu(self.conv1(x)) #(batch_size, chanel=128, num_points)
        x = F.relu(self.conv2(x)) #(batch_size, chanel=256, num_points)
        x = F.relu(self.conv3(x))
        x = self.max_pool(x)
        x = x.view(-1, self.num_points)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)
        return x

class PointNet_global_feat(nn.Module):
    def __init__(self, num_points=1024):
        super(PointNet_global_feat, self).__init__()

        self.num_points = num_points
        
        self.conv1 = torch.nn.Conv1d(3, 128, 1)
        self.conv2 = torch.nn.Conv1d(128, 256, 1)
        self.conv3 = torch.nn.Conv1d(256, self.num_points, 1)
        self.max_pool = nn.MaxPool1d(self.num_points)

        self.bn1 = nn.BatchNorm1d(128)
        self.bn2 = nn.BatchNorm1d(256)
        self.bn3 = nn.BatchNorm1d(1024)

    def forward(self, x):
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = self.max_pool(x)
        x = x.view(-1, 1024)

        return x


class STN3d(nn.Module):
    def __init__(self, quaternion=False):
        super(STN3d, self).__init__()
        self.quaternion = quaternion
        self.conv1 = torch.nn.Conv1d(3, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        if self.quaternion:
            self.fc3 = nn.Linear(256, 4)
        else:
            self.fc3 = nn.Linear(256, 9)
        self.relu = nn.ReLU()

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)


    def forward(self, x):
        batchsize = x.size()[0]
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)
        # print("********************************************")
        # print(x.shape)
        x = F.relu(self.bn4(self.fc1(x)))
        # x = F.relu(self.fc1(x))
        x = F.relu(self.bn5(self.fc2(x)))
        # x = F.relu(self.fc2(x))
        x = self.fc3(x)

        if self.quaternion:
            iden = Variable(torch.FloatTensor([1, 0, 0, 0]))
            if x.is_cuda:
                iden = iden.cuda()
            x = x + iden
            if x.is_cuda:
                trans = Variable(torch.cuda.FloatTensor(batchsize, 3, 3))
            else:
                trans = Variable(torch.FloatTensor(batchsize, 3, 3))
            x = utils.batch_quat_to_rotmat(x, trans)
        else:
            #iden = Variable(torch.from_numpy(np.array([1,0,0,0,1,0,0,0,1]).astype(np.float32))).view(1,9).repeat(batchsize,1)
            #if x.is_cuda:
                #iden = iden.cuda()
            #x = x + iden
            x = x.view(-1, 3, 3)
        return x


class STNkd(nn.Module):
    def __init__(self, k=64):
        super(STNkd, self).__init__()
        self.conv1 = torch.nn.Conv1d(k, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.fc1 = nn.Linear(1024, 512)
        self.fc2 = nn.Linear(512, 256)
        self.fc3 = nn.Linear(256, k*k)
        self.relu = nn.ReLU()

        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.bn4 = nn.BatchNorm1d(512)
        self.bn5 = nn.BatchNorm1d(256)

        self.k = k

    def forward(self, x):
        batchsize = x.size()[0]
        x = F.relu(self.bn1(self.conv1(x)))
        x = F.relu(self.bn2(self.conv2(x)))
        x = F.relu(self.bn3(self.conv3(x)))
        x = torch.max(x, 2, keepdim=True)[0]
        x = x.view(-1, 1024)
        # print("fffff")

        #x = F.relu(self.bn4(self.fc1(x)))
        x = F.relu(self.fc1(x))
        #x = F.relu(self.bn5(self.fc2(x)))
        x = F.relu(self.fc2(x))
        x = self.fc3(x)

        iden = Variable(torch.from_numpy(np.eye(self.k).flatten().astype(np.float32))).view(1,self.k*self.k).repeat(batchsize,1)
        if x.is_cuda:
            iden = iden.cuda()
        x = x + iden
        x = x.view(-1, self.k, self.k)
        return x

class PointNet_feat_segmentation(nn.Module):
    def __init__(self, global_feat = False, feature_transform = True):
        super(PointNet_feat_segmentation, self).__init__()
        self.stn = STN3d()
        self.conv1 = torch.nn.Conv1d(3, 64, 1)
        self.conv2 = torch.nn.Conv1d(64, 128, 1)
        self.conv3 = torch.nn.Conv1d(128, 1024, 1)
        self.bn1 = nn.BatchNorm1d(64)
        self.bn2 = nn.BatchNorm1d(128)
        self.bn3 = nn.BatchNorm1d(1024)
        self.global_feat = global_feat
        self.feature_transform = feature_transform
        if self.feature_transform:
            self.fstn = STNkd(k=64)

    def forward(self, x):
        # print("171")
        # print(x)
        n_pts = x.size()[2] #getting number of point_cloud (dataset structure: batch_size ch point_cloud)
        # print("173")
        trans = self.stn(x) #T-Net
        # print(trans)
        # print("175")
        x = x.transpose(2, 1) #transpose for matrix multiplication
        # print("177")
        x = torch.bmm(x, trans) #matrix multiplication 
        # print("179")
        # print(x)
        x = x.transpose(2, 1)
        # print("181")
        x = F.relu(self.bn1(self.conv1(x)))
        # print("self_transform" + str(self.feature_transform))

        if self.feature_transform:
            # print("204")
            trans_feat = self.fstn(x)
            # print("205")
            x = x.transpose(2,1)
            # print("207")
            x = torch.bmm(x, trans_feat)
            # print("209")
            x = x.transpose(2,1)
            # print("211")
        else:
            trans_feat = None

        pointfeat = x
        x = F.relu(self.bn2(self.conv2(x)))
        x = self.bn3(self.conv3(x))
        x = torch.max(x, 2, keepdim=True)[0] #getting max data of each ch  (※:[0] is role for getting maxed data([1] is index))
        x = x.view(-1, 1024)

        if self.global_feat:
            return x, trans, trans_feat
        else:
            x = x.view(-1, 1024, 1).repeat(1, 1, n_pts) #set number of point_cloud for cat (********)
            return torch.cat([x, pointfeat], 1) #convoluted point_cloud(concat final_encoded_data and data_passed_T-net (purpose:add ch_data)) and first T-net second T-Net

