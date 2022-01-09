#! /bin/env python2
# -*- coding: utf-8 -*-

from numpy.core.fromnumeric import size
import torch
import numpy as np
# from torch._C import R
from . import networks
from os.path import join
from utils.util import print_network
from utils import util
import h5py


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
        # self.checkpoints_process_swich = opt.checkpoints_process_swich
        self.dataset_mode = opt.dataset_mode
        self.save_dir = join(self.checkpoints_dir, self.dataset_mode, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.local_save_dir=join(self.local_checkpoints_dir, self.dataset_mode, self.checkpoints_human_swich, self.arch, self.concat_dataset_model)
        self.gpu_ids = opt.gpu_ids
        self.device = torch.device('cuda:{}'.format(self.gpu_ids[0])) if self.gpu_ids else torch.device('cpu')
        self.is_train = self.opt.is_train
        self.instance_number_manual = opt.instance_number-1
        print("opt_phase is " + str(self.opt.phase))
        print("process + " + str(self.process_swich))
        self.instance_number = opt.instance_number
        self.dataset_mode = opt.dataset_mode

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
        

            if self.dataset_mode == "pose_estimation":
                y_data = torch.from_numpy(data["y_data"].astype(np.float32))
                x_data = x_data.transpose(2, 1)
            elif self.dataset_mode == "instance_segmentation":
                y_data = torch.from_numpy(data["y_data"].astype(np.float32))
                x_data = x_data.transpose(2, 1)
            elif self.dataset_mode == "semantic_segmentation":
                y_data = torch.from_numpy(data["y_data"].astype(np.int64))
                x_data = x_data.transpose(2, 1)

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

    def set_input_acc(self, data):
        x_data = torch.from_numpy(data["x_data"].astype(np.float32)) 
        if self.dataset_mode == "pose_estimation":
            y_data = torch.from_numpy(data["y_data"].astype(np.float32))
            x_data = x_data.transpose(2, 1)
        elif self.dataset_mode == "instance_segmentation":
            y_data = torch.from_numpy(data["y_data"].astype(np.float32))
            x_data = x_data.transpose(2, 1)
        elif self.dataset_mode == "semantic_segmentation":
            y_data = torch.from_numpy(data["y_data"].astype(np.int64))
            x_data = x_data.transpose(2, 1)

        self.x_data, self.y_data = x_data.to(self.device), y_data.to(self.device)

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
        # pred = self.net(self.x_data)

        if self.process_swich == "raugh_recognition":
            pred = self.net(self.x_data)
            self.loss = self.criterion(pred, self.y_data)
        elif self.process_swich == "object_segment":
            if self.arch == "JSIS3D":
                pred = self.net(self.x_data)
                self.loss = self.criterion(pred, self.y_data, self.instance_number)
            elif self.arch == "PointNet_Segmentation":
                pred, trans_feat = self.net(self.x_data)
                # print("train")
                # print(type(pred))
                self.loss = self.criterion(pred, self.y_data, trans_feat)
        self.loss.backward()
        self.optimizer.step()
        # return self.loss.item() * self.x_data.size(0)
        return self.loss.item()


    def val_step(self):
        self.net.eval()
        if self.process_swich == "raugh_recognition":
            pred = self.net(self.x_data)
            self.loss = self.criterion(pred, self.y_data)
        elif self.process_swich == "object_segment":
            if self.arch == "JSIS3D":
                pred = self.net(self.x_data)
                self.loss = self.criterion(pred, self.y_data, self.instance_number)
            if self.arch == "PointNet_Segmentation":
                pred, trans_feat = self.net(self.x_data)
                # print("test")
                # print(type(pred))
                self.loss = self.criterion(pred, self.y_data, trans_feat)
        # return self.loss.item() * self.x_data.size(0)
        return self.loss.item()


    def test_step(self):
        if self.process_swich == "raugh_recognition":
            # print(self.x_data.shape)
            for i in range(4):
                print(str(self.x_data[0][0][i]) +  " " + str(self.x_data[0][1][i]) +  " " + str(self.x_data[0][2][i]))
                print("")
            pred = self.net(self.x_data)
            # print("p")
            # print(pred.shape)
            pred = pred.to('cpu').detach().numpy().copy()
            print("****")
            print(type(pred))
            print(pred.shape)
            for i in range(4):
                print(str(pred[0][0]) +  " " + str(pred[0][1]) +  " " + str(pred[0][2]) +  " " + str(pred[0][3]) +  " " + str(pred[0][5]))
                print(str(pred[0][6]) +  " " + str(pred[0][7]) +  " " + str(pred[0][8]) +  " " + str(pred[0][9]) +  " " + str(pred[0][10]) +  " " + str(pred[0][11]))
                print("")
             # print("****")
            # print(type(pred))
            # print(pred.shape)
            # for i in range(4):
            #     print(str(pred[0][0]) +  " " + str(pred[0][1]) +  " " + str(pred[0][3]) +  " " + str(pred[0][4]) +  " " + str(pred[0][5]))
            #     print(str(pred[0][6]) +  " " + str(pred[0][7]) +  " " + str(pred[0][8]) +  " " + str(pred[0][9]) +  " " + str(pred[0][10]) +  " " + str(pred[0][11]))
            #     print("")
            # print("pred")
            # print(pred.shape)
        elif self.process_swich == "object_segment":
            if self.arch == "JSIS3D":
                pred = self.net(self.x_data)
                pred = pred.to('cpu').detach().numpy().copy()
            if self.arch == "PointNet_Segmentation":
                pred, trans = self.net(self.x_data)
                # print("output")
                # print(pred.shape)
                # for i in pred:
                #     ppi = i
                # pred = ppi.to('cpu').detach().numpy().copy()
                pred = pred.contiguous().cpu().data.max(2)[1].numpy()
                # print(pred.shape)
                # pred = pred.to('cpu').detach().numpy().copy()
        return pred


    def acc_step(self):
        if self.process_swich == "raugh_recognition":
            pred = self.net(self.x_data)
            # print("p")
            # print(pred.shape)
            pred = pred.to('cpu').detach().numpy().copy()
            # print("pred")
            # print(pred.shape)
        elif self.process_swich == "object_segment":
            if self.arch == "JSIS3D":
                pred = self.net(self.x_data)
                pred = pred.to('cpu').detach().numpy().copy()
            if self.arch == "PointNet_Segmentation":
                pred, trans = self.net(self.x_data)
                # print("output")
                # print(pred.shape)
                # for i in pred:
                #     ppi = i
                # pred = ppi.to('cpu').detach().numpy().copy()
                pred = pred.contiguous().cpu().data.max(2)[1].numpy()
                # print(pred.shape)
                # pred = pred.to('cpu').detach().numpy().copy()
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
        # print("load_state_patg")
        # print(self.net)
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
        # net.load_state_dict(state_dict["model_state_dict"])

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

    def progress_save_pcd(self, opt, epoch, index):
        if self.process_swich == "raugh_recognition":
            pred = self.net(self.x_data)
            pred = pred.to('cpu').detach().numpy().copy()
        elif self.process_swich == "object_segment":
            if self.arch == "JSIS3D":
                pred = self.net(self.x_data)
                pred = pred.to('cpu').detach().numpy().copy()
            if self.arch == "PointNet_Segmentation":
                pred, trans = self.net(self.x_data)
                print("net")
                print(pred.shape)
                print(pred)
                pred = pred.contiguous().cpu().data.max(2)[1].numpy()
        
        self.x_data = self.x_data.to('cpu').detach().numpy().copy()
        self.y_data = self.y_data.to('cpu').detach().numpy().copy()
        self.concat_dataset_model = '+'.join(opt.dataset_model)
        pcd_dir = "/home/ericlab/DENSO_results/August/pcl_visu/progress_output/"+opt.dataset_mode+"/"+self.concat_dataset_model +"/"+str(epoch)
        util.mkdir(pcd_dir)
        result_h5 = h5py.File(pcd_dir + "/result" + str(epoch) + ".hdf5", "a")
        data = result_h5.create_group("data_" + str(index))
        data.create_dataset("Points", data=self.x_data, compression="lzf")
        data.create_dataset("est", data=pred, compression="lzf")
        data.create_dataset("ground_truth", data=self.y_data, compression="lzf")
        result_h5.flush()
       
        # f = open(pcd_dir+"/result"+str(index)+".txt", 'a')
        # print("pred")
        # print(pred.shape)
        # print(pred)
        # f.write(str(pred))
        # print("output")
        # print(pred.shape)
        # print(type(pred))
        
        # for i in pred:
        #     ppi = i
        # pred = ppi.to('cpu').detach().numpy().copy()
        # pred = pred.contiguous().cpu().data.max(2)[1].numpy()
        # pred = pred.to('cpu').detach().numpy().copy()
        # print("pred")
        # print(pred.shape)

        # for i in range(pred.shape[0]):
        #     for j in range(pred.shape[1]):
        #         # pred = pred.contiguous().cpu().data.max(2)[1].numpy()
        #         f = open("/home/ericlab/pcl_visu/progress_output/"+"result"+str(epoch)+"_"+str(i)+".txt", 'a')
        #         f.write(str(pred[i,j])+"\n")
        # for i in range(pred.shape[0]):
        #     for j in range(pred.shape[1]):
        #         f.write(str(pred[i,j])+"\n")

        # pcl_visu = pcl.PointCloud(pred)
        # pcl.save(pcl_visu, "/home/ericlab/pcl_visu/progress_output/"+"result"+str(epoch)+".pcd")
        # pred = pred.to('cpu').detach().numpy().copy()
        return pred

