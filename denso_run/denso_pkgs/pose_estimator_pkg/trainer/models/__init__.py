from .pose_estimate import EstimatorModel
import torch


def create_model(opt):
    model = torch.nn.DataParallel(opt)
    #cudnn.benchmark = False
    model = EstimatorModel(opt)
    
    return model
