#! /bin/env python2
# -*- coding: utf-8 -*-

import torch
import numpy as np
from . import networks
from os.path import join
from utils.util import print_network


class EstimatorModel:
    def __init__(self, opt):
        self.opt = opt
        self.name = opt.name
        self.checkpoints_dir = opt.checkpoints_dir
        self.local_checkpoints_dir=opt.local_checkpoints_dir
        self.dataset_model = self.opt.dataset_model
        self.concat_dataset_model = '+'.join(self.opt.dataset_model)
        self.name=opt.name
        self.checkpoints_swich=opt.checkpoints_swich
        self.save_dir = join(self.checkpoints_dir, self.checkpoints_swich,self.name, self.concat_dataset_model)
        self.local_save_dir=join(self.local_checkpoints_dir, self.checkpoints_swich,self.name, self.concat_dataset_model)
        self.gpu_ids = opt.gpu_ids
        self.device = torch.device('cuda:{}'.format(self.gpu_ids[0])) if self.gpu_ids else torch.device('cpu')
        self.is_train = opt.is_train

        self.optimizer = None
        self.x_data  = None
        self.y_data = None
        self.position_offset = None
        self.loss = None

        self.net = networks.define_network(opt)
        self.criterion = networks.define_loss(opt).to(self.device)

        if self.is_train:
            self.net.train(self.is_train)
            self.optimizer = torch.optim.Adam(self.net.parameters(), lr=opt.lr)
            print_network(self.net)

        if not self.is_train:
            self.load_network_estimator(opt.which_epoch)


    def get_centroid(self, data):
        x_mean = torch.mean(data, axis=1)
        x_mean = x_mean.unsqueeze(1)
        self.position_offset = x_mean
        data = data - x_mean
        return data


    def set_input(self, data):
        if self.opt.phase == "train":
            x_data = torch.from_numpy(data["x_data"].astype(np.float32))
            y_data = torch.from_numpy(data["y_data"].astype(np.float32)) 
        

            if self.name == "PointNet":
                x_data = x_data.transpose(2, 1)

            self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

        elif self.opt.phase == "test":
            x_data = torch.from_numpy(data)
            x_data = x_data.float()

            if self.name == "PointNet":
                #x_data = self.get_centroid(x_data)
                x_data = x_data.transpose(2, 1)

            self.x_data = x_data.to(self.device)


    def train_step(self):
        self.net.train()
        self.optimizer.zero_grad()
        pred = self.net(self.x_data)

        self.loss = self.criterion(pred, self.y_data)
        self.loss.backward()
        self.optimizer.step()
        return self.loss.item() * self.x_data.size(0)


    def val_step(self):
        self.net.eval()
        pred = self.net(self.x_data)
        self.loss = self.criterion(pred, self.y_data)
        return self.loss.item() * self.x_data.size(0)


    def test_step(self):
        pred = self.net(self.x_data)
        pred = pred.to('cpu').detach().numpy().copy()
        return pred


    def load_network(self, which_epoch):
        #save_filename = "%s_net-has.pth" % which_epoch
        #self.save_dir = "/home/ericlab/MEGAsync/TEI_PC/3_24-6layer/PointNet/dataset_20000_1.hdf5"
        save_filename = "latest_net.pth"
        #self.save_dir = "/home/ericlab/Rikuken/Mega/X10/PointNet/dataset_20000.hdf5"
        #self.save_dir = "/home/ericlab/MEGAsync/X10/03_20/PointNet/dataset_20000_1.hdf5"
        #save_filename = "latest_net.pth"
        #self.save_dir = "/home/ericlab/MEGAsync/TEI_PC/3_24-6layer/PointNet/dataset_20000_1.hdf5"
        #load_path = join(self.save_dir, PC_NAME,save_filename)
        load_path = join(self.save_dir, save_filename)
        net = self.net

        if isinstance(net, torch.nn.DataParallel):
            net = net.module
        
        print("loading the model from %s" % load_path)
        state_dict = torch.load(load_path, map_location=str(self.device))
        if hasattr(state_dict, "_metadata"):
            del state_dict._metadata
        net.load_state_dict(state_dict,strict=False)

    def load_network_estimator(self, which_epoch):
        #save_filename = "%s_net-has.pth" % which_epoch
        #self.save_dir = "/home/ericlab/MEGAsync/TEI_PC/3_24-6layer/PointNet/dataset_20000_1.hdf5"
        save_filename = self.checkpoints_dir
        #self.save_dir = "/home/ericlab/Rikuken/Mega/X10/PointNet/dataset_20000.hdf5"
        #self.save_dir = "/home/ericlab/MEGAsync/X10/03_20/PointNet/dataset_20000_1.hdf5"
        #save_filename = "latest_net.pth"
        #self.save_dir = "/home/ericlab/MEGAsync/TEI_PC/3_24-6layer/PointNet/dataset_20000_1.hdf5"
        #load_path = join(self.save_dir, PC_NAME,save_filename)
        load_path = save_filename
        net = self.net

        if isinstance(net, torch.nn.DataParallel):
            net = net.module
        #load_path = "/home/ericlab/OneDrive/DENSO/raugh_recognition/checkpoint/onoyama/0423/PointNet/dataset_20000.hdf5/latest_net.pth"
        print("loading the model from %s" % load_path)
        state_dict = torch.load(load_path, map_location=str(self.device))
        if hasattr(state_dict, "_metadata"):
            del state_dict._metadata
        net.load_state_dict(state_dict,strict=False)

    def save_network(self, which_epoch):
        save_filename= "%s_net.pth" % (which_epoch)
        save_path = join(self.save_dir, save_filename)
        local_save_path=join(self.local_save_dir,save_filename)
        if len(self.gpu_ids) > 0 and torch.cuda.is_available():
            torch.save(self.net.module.cpu().state_dict(), save_path)
            torch.save(self.net.module.cpu().state_dict(), local_save_path)
            self.net.cuda(self.gpu_ids[0])
        else:
            torch.save(self.net.cpu().state_dict(), save_path)

