import torch 
import torchvision.models as models
<<<<<<< HEAD
pth_file = '/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/estimator/weights/PointNet/HV8/latest_net_original.pth'
=======
<<<<<<< HEAD
pth_file = '/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/estimator/weights/PointNet/HV8/latest_net.pth'
=======
pth_file = '/home/tsuchidashinya/ros_package/denso_ws/src/denso_run/denso_pkgs/pose_estimator_pkg/estimator/weights/PointNet/HV8/latest_net_original.pth'
>>>>>>> b8a4e201324bbf227462141351058788a24b490f
>>>>>>> e6cc65836113879060692d92bd2bc7d3953d678e
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
