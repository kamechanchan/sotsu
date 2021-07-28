#! /bin/env python2
# -*- coding: utf-8 -*-

from numpy.core.fromnumeric import size
import torch
import numpy as np
from . import networks
from os.path import join
from utils.util import print_network


class EstimatorModel:
    def __init__(self, opt):
        self.opt = opt
        self.process_swich = opt.process_swich
        self.checkpoints_dir = opt.checkpoints_dir
        self.local_checkpoints_dir=opt.local_checkpoints_dir
        self.dataset_model = self.opt.dataset_model
        self.concat_dataset_model = '+'.join(self.opt.dataset_model)
        self.arch = opt.arch
        self.checkpoints_human_swich = opt.checkpoints_human_swich
        self.checkpoints_process_swich = opt.checkpoints_process_swich
        self.save_dir = join(self.checkpoints_dir, self.checkpoints_process_swich, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.local_save_dir=join(self.local_checkpoints_dir, self.checkpoints_process_swich, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.gpu_ids = opt.gpu_ids
        self.device = torch.device('cuda:{}'.format(self.gpu_ids[0])) if self.gpu_ids else torch.device('cpu')
        self.is_train = self.opt.is_train
        self.instance_number_manual = opt.instance_number-1
        print("opt_phase is " + str(self.opt.phase))
        print("process + " + str(self.process_swich))

        self.optimizer = None
        self.x_data  = None
        self.y_data = None
        self.position_offset = None
        self.loss = None

        self.net = networks.define_network(opt)
        self.criterion = networks.define_loss(opt).to(self.device)
        print('is_train is ' + str(self.is_train))
        if self.is_train:
            self.net.train(self.is_train)
            self.optimizer = torch.optim.Adam(self.net.parameters(), lr=self.opt.lr)
            print_network(self.net)

        if not self.is_train:
            self.load_network_estimator(opt.which_epoch)
            #self.load_network(opt.which_epoch)


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
            # self.instance_number = torch.from_numpy(data["sizes"].astype(np.int32))
            x_data = x_data.transpose(2, 1)
            # if self.process_swich == "raugh_recognition":
            #     x_data = x_data.transpose(2, 1)
            # elif self.process_swich == "object_segment":
            #     x_data = x_data.transpose(2, 1)

            # print("datadata")
            # print(x_data.type)
            # print(y_data.type)
            # print(instance_number.type)

            self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

        elif self.opt.phase == "test":
            x_data = torch.from_numpy(data)
            x_data = x_data.float()

            if self.process_swich == "raugh_recognition":
                #x_data = self.get_centroid(x_data)
                x_data = x_data.transpose(2, 1)
            elif self.process_swich == "object_segment":
                x_data = x_data.transpose(2, 1)

            self.x_data = x_data.to(self.device)


    def set_input_segmentation(self, data):
        x_data = data["x_data"]
        y_data = data["y_data"]
        sizes = data["sizes"]
        if self.opt.phase == "train":
            # x_data = np.array(meta_x)
            # y_data = np.array(meta_y)
            x_data = torch.from_numpy(x_data.astype(np.float32))
            y_data = torch.from_numpy(y_data.astype(np.float32))
            sizes = torch.from_numpy(sizes.astype(np.int32))
            # self.instance_number = torch.from_numpy(data["sizes"].astype(np.int32))
            
            x_data = x_data.transpose(2, 1)
            # y_data = np.array(y_data)
            # if self.process_swich == "raugh_recognition":
            #     x_data = x_data.transpose(2, 1)
            # elif self.process_swich == "object_segment":
            #     x_data = x_data.transpose(2, 1)

            # print("datadata")
            # print(x_data.type)
            # print(y_data.type)
            # print(instance_number.type)

            self.x_data, self.y_data, self.sizes = x_data.to(self.device), y_data.to(self.device), sizes.to(self.device)
            # self.x_data = x_data.to(self.device)
            # self.y_data = y_data
            # self.y_data = [ann.to(self.device) for ann in y_data]
            # print(type(y_data))

        elif self.opt.phase == "test":
            # x_data = np.array(meta_x)
            x_data = torch.from_numpy(x_data)
            # x_data = x_data.float()

            if self.process_swich == "raugh_recognition":
                #x_data = self.get_centroid(x_data)
                x_data = x_data.transpose(2, 1)
            elif self.process_swich == "object_segment":
                x_data = x_data.transpose(2, 1)

            self.x_data = x_data.to(self.device)


    def train_step(self):
        self.net.train()
        self.optimizer.zero_grad()
        pred = self.net(self.x_data)

        if self.process_swich == "raugh_recognition":
            self.loss = self.criterion(pred, self.y_data)
        elif self.process_swich == "object_segment":
            # self.loss = self.criterion(pred, self.y_data, self.instance_number_manual)
            # self.loss = self.criterion(pred, self.y_data, np.shape(self.y_data)[2])
            self.loss = self.criterion(pred, self.y_data, self.sizes)
            # print("data")
            # print(self.y_data.shape[2])
            # print("loss")
            # print(self.loss)
        self.loss.backward()
        self.optimizer.step()
        return self.loss.item() * self.x_data.size(0)


    def val_step(self):
        self.net.eval()
        pred = self.net(self.x_data)
        if self.process_swich == "raugh_recognition":
            self.loss = self.criterion(pred, self.y_data)
        elif self.process_swich == "object_segment":
            # self.loss = self.criterion(pred, self.y_data, self.instance_number_manual)
            # self.loss = self.criterion(pred, self.y_data, self.y_data.shape[2])
            self.loss = self.criterion(pred, self.y_data, self.sizes)
        return self.loss.item() * self.x_data.size(0)


    def test_step(self):
        pred = self.net(self.x_data)
        pred = pred.to('cpu').detach().numpy().copy()
        return pred


    def load_network(self, which_epoch):
        save_filename = "latest_net.pth"
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
        save_filename = self.checkpoints_dir
        load_path = save_filename
        net = self.net

        if isinstance(net, torch.nn.DataParallel):
            net = net.module
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

