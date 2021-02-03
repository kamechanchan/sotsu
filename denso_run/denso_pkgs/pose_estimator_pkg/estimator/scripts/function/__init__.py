from geometry_msgs.msg import TransformStamped

def num2ros_transform(pos, ori):
    trans = TransformStamped()
    trans.transform.translation.x = pos[0]
    trans.transform.translation.y = pos[1]
    trans.transform.translation.z = pos[2]
    trans.transform.rotation.x = ori[0]
    trans.transform.rotation.y = ori[1]
    trans.transform.rotation.z = ori[2]
    trans.transform.rotation.w = ori[3]
    return trans

def predict_pose(model, data, arg):
    if arg == "PointNet":
        from pointnet_est import *
        y, est_time = pose_prediction(model, data)
        return y, est_time

    elif arg == "C3D_Voxel":
        from voxel_est import *
        y, est_time = pose_prediction(model, data)
        return y, est_time
    else:
        print("Cloud not predict DNN arch. __init_.py Error!")

