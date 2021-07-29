import argparse
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), '../'))

from utils import util
import torch


class BaseOptions:
    def __init__(self):
        self.parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
        self.initialized = False
        self.dataset_number = 1

    def initialize(self):
        self.parser.add_argument('--main_directory',type=str,default=__file__)
        self.parser.add_argument('--dataset_mode', choices={"instance_segmentation", "pose_estimation"}, default='pose_estimation')
        self.parser.add_argument('--dataset_model', nargs=self.dataset_number, type=str, default='HV8')
        self.parser.add_argument('--max_dataset_size', nargs=self.dataset_number, type=int, default=float("inf"), help='Maximum num of samples per epoch')
        self.parser.add_argument('--process_swich', type=str, choices={"raugh_recognition", "object_segment"}, default="debug")
        self.parser.add_argument('--batch_size', type=int, default=8)
        self.parser.add_argument('--num_epoch', type=int, default=150)
        self.parser.add_argument('--arch', type=str, default="debug")
        self.parser.add_argument('--resolution', type=int, default=1024)
        self.parser.add_argument('--gpu_ids', type=str, default='-1')
        self.parser.add_argument('--gpu_num', type=int, default=0)
        self.parser.add_argument('--num_threads', type=int, default=3)
        self.parser.add_argument('--checkpoints_dir', type=str, default="/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/weights")
        self.parser.add_argument('--serial_batches', action='store_true', help='if true, takes meshs in order')
        self.parser.add_argument('--export_folder', type=str, default='exports intermediate collapses to this folder')
        self.parser.add_argument('--checkpoints_human_swich',type=str,default='ishiyama')
        self.parser.add_argument('--dataroot_swich',type=str,default='front')
        self.parser.add_argument('--local_checkpoints_dir',type=str,default='/home/ericlab/DENSO/raugh_recognition/checkpoint')
        self.parser.add_argument('--local_export_folder', type=str, default='exports intermediate collapses to this folder')
        self.parser.add_argument('--tensorboardX_results_directory',type=str,default="/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/trainer/tensorboardX/")
        self.parser.add_argument('--tensorboardX_results_directory_switch',type=str,default="ishiyama")
        self.parser.add_argument('--dataset_number', type=int, default=self.dataset_number)
        self.parser.add_argument('--lr', type=float, default=0.001, help='initial learning rate of adam')
        self.parser.add_argument('--is_train', type=bool, default=False)
        # for instance-segmentation
        self.parser.add_argument('--embedded_size', type=int, default=32)
        self.parser.add_argument('--delta_d', type=float, default=1.5)
        self.parser.add_argument('--delta_v', type=float, default=0.5)
        self.parser.add_argument('--instance_number', type=int, default=8)
        self.parser.add_argument('--checkpoints_process_swich',type=str,default='raugh_recognition')

        self.initialized = True


    def parse(self):
        if not self.initialized:
            self.initialize()
        self.opt, unknown = self.parser.parse_known_args()
        str_ids = self.opt.gpu_ids.split(',')
        self.opt.gpu_ids = []

        self.concat_dataset_model = '+'.join(self.opt.dataset_model)

        for str_id in str_ids:
            id = int(str_id)
            # print("*************************************")
            if id>= 0:
                self.opt.gpu_ids.append(id)
                # print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
                # print(self.opt.gpu_ids)

        if len(self.opt.gpu_ids) > 0:
            torch.cuda.set_device(self.opt.gpu_ids[0])

        args = vars(self.opt)

        if self.opt.export_folder:
            self.opt.export_folder = os.path.join(self.opt.checkpoints_dir, self.opt.checkpoints_process_swich, self.opt.checkpoints_human_swich, self.opt.arch, self.concat_dataset_model, self.opt.export_folder)
            util.mkdir(self.opt.export_folder)

        if self.opt.local_export_folder:
            self.opt.local_export_folder = os.path.join(self.opt.local_checkpoints_dir, self.opt.checkpoints_process_swich, self.opt.checkpoints_human_swich, self.opt.arch, self.concat_dataset_model, self.opt.local_export_folder)
            util.mkdir(self.opt.export_folder)

        if self.opt.is_train:
            print("---------------Options-------------")
            for k, v in sorted(args.items()):
                print('%s: %s' % (str(k), str(v)))
            print("---------------End-------------")

            expr_dir = os.path.join(self.opt.checkpoints_dir, self.opt.checkpoints_process_swich, self.opt.checkpoints_human_swich, self.opt.arch, self.concat_dataset_model)
            local_expr_dir = os.path.join(self.opt.local_checkpoints_dir, self.opt.checkpoints_process_swich, self.opt.checkpoints_human_swich, self.opt.arch, self.concat_dataset_model)
            util.mkdir(expr_dir)
            util.mkdir(local_expr_dir)

            file_name = os.path.join(expr_dir, "opt.txt")
            with open(file_name, "wt") as opt_file:
                opt_file.write("------------Options------------\n")
                for k, v in sorted(args.items()):
                    opt_file.write('%s: %s\n' % (str(k), str(v)))
                opt_file.write("-------------End---------------\n")

            local_file_name = os.path.join(local_expr_dir, "opt.txt")
            with open(local_file_name, "wt") as opt_file:
                opt_file.write("------------Options------------\n")
                for k, v in sorted(args.items()):
                    opt_file.write('%s: %s\n' % (str(k), str(v)))
                opt_file.write("-------------End---------------\n")

        return self.opt
