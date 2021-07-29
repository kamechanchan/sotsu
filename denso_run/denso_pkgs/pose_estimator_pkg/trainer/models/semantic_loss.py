import torch
import torch.nn.functional as F

class Semantic_Loss(torch.nn.Module):
    def __init__(self, mat_diff_loss_scale=0.001):
        super(Semantic_Loss, self).__init__()
        self.mat_diff_loss_scale = mat_diff_loss_scale

    def forward(self, pred, target, trans_feat):
        print("gks")
        print(target.shape)
        print(pred.shape)
        loss = torch.nn.NLLLoss(pred, target)
        mat_diff_loss = self.feature_transform_reguliarzer(trans_feat)
        total_loss = loss + mat_diff_loss * self.mat_diff_loss_scale
        return total_loss

    def feature_transform_reguliarzer(trans):
        d = trans.size()[1]
        I = torch.eye(d)[None, :, :]
        if trans.is_cuda:
            I = I.cuda()
        loss = torch.mean(torch.norm(torch.bmm(trans, trans.transpose(2, 1)) - I, dim=(1, 2)))
        return loss