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


