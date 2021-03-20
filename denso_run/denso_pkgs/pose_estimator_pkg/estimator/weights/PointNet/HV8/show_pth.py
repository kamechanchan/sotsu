import torch 
import torchvision.models as models



pth_file = '/home/ericlab/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/estimator/weights/PointNet/HV8/latest_net_original.pth'
#pretrained_dict =torch.load(pth_file)
#print('state:',type(pretrained_dict))
#net = models.latest_net(pretrained=True)
net = torch.load(pth_file)
#net.load_state_dict(torch.load(pth_file))
print('.pth type:', type(net))
print('.pth len:',len(net))
for i in net.keys():
    print(i, type(net[i]), net[i].shape)
#print(net)
