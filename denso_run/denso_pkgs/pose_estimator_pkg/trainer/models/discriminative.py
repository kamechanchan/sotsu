from __future__ import print_function
from numpy.core.fromnumeric import size
import torch
import torch.nn as nn
import numpy as np


import torch
import torch.nn as nn


class DiscriminativeLoss(nn.Module):
    def __init__(self, delta_d, delta_v,
                 alpha=1.0, beta=1.0, gamma=0.001,
                 reduction='mean'):
        # TODO: Respect the reduction rule
        super(DiscriminativeLoss, self).__init__()
        self.alpha = alpha
        self.beta = beta
        self.gamma = gamma
        # Set delta_d > 2 * delta_v
        self.delta_d = delta_d
        self.delta_v = delta_v

    def forward(self, embedded, masks, size):
        centroids, size_ex = self._centroids(embedded, masks, size)
        L_v = self._variance(embedded, masks, centroids, size_ex)
        L_d = self._distance(centroids, size_ex)
        L_r = self._regularization(centroids, size_ex)
        loss = self.alpha * L_v + self.beta * L_d + self.gamma * L_r
        return loss

    def _centroids(self, embedded, masks, size):
        batch_size = embedded.size(0)
        embedding_size = embedded.size(2)
        # print("masks")
        # print(masks.shape)
        K = masks.size(2)
        # print("kkkkkkkk")
        #print(embedded.shape)
        #print(size.shape)
        # print(masks.shape)
        x = embedded.unsqueeze(2).expand(-1, -1, K, -1)
        masks = masks.unsqueeze(3)
        print("**********")
        print(x.shape)
        print(masks.shape)
        x = x * masks
        centroids = []
        size_ex = [size] * batch_size
        for i in range(batch_size):
            # n = size[i]
            n = size
            # print("nnnnnnnnnnnnn")
            # print(n)
            # print(x[i,:,:n].sum(0))
            # print(masks[i,:,:n].sum(0))
            mu = x[i,:,:n].sum(0) / masks[i,:,:n].sum(0)
            # print(mu.size())

            mask_delete = masks[i,:,:n].sum(0)
            cnt = 0
            for j in range(n):
                if mask_delete[j] == 0:
                    # print(mu)
                    # mu[j, :] = 0
                    mu = mu.to("cpu").detach().numpy().copy()
                    mu = np.delete(mu, j, 0)
                    mu = torch.from_numpy(mu.astype(np.float32)).clone()
                    filled = torch.zeros(1, embedding_size)
                    filled = filled.to(mu.device)
                    mu = torch.cat([mu, filled], dim=0)
                    mu = mu.to(embedded.device)
                    cnt += 1
                    # print(mu)
            size_ex[i] = size_ex[i] - cnt
            # if K > n:
            #     m = int(K - n)
            #     filled = torch.zeros(m, embedding_size)
            #     filled = filled.to(embedded.device)
            #     mu = torch.cat([mu, filled], dim=0)
            # print(mu)
            centroids.append(mu)
        centroids = torch.stack(centroids)
        return centroids, size_ex

    def _variance(self, embedded, masks, centroids, size):
        batch_size = embedded.size(0)
        num_points = embedded.size(1)
        embedding_size = embedded.size(2)
        K = masks.size(2)
        # Convert input into the same size
        mu = centroids.unsqueeze(1).expand(-1, num_points, -1, -1)
        x = embedded.unsqueeze(2).expand(-1, -1, K, -1)
        # Calculate intra pull force
        var = torch.norm(x - mu, 2, dim=3) #L2Nolm
        var = torch.clamp(var - self.delta_v, min=0.0) ** 2
        #print("sizesize")
        #print(var.shape)
        var = var * masks
        loss = 0.0
        for i in range(batch_size):
            n = size[i]
            # n = size
            loss += torch.sum(var[i,:,:n]) / torch.sum(masks[i,:,:n])
        loss /= batch_size
        return loss

    def _distance(self, centroids, size):
        batch_size = centroids.size(0)
        loss = 0.0
        for i in range(batch_size):
            n = size[i]
            # n = size
            if n <= 1: continue
            mu = centroids[i, :n, :]
            mu_a = mu.unsqueeze(1).expand(-1, n, -1)
            mu_b = mu_a.permute(1, 0, 2)
            diff = mu_a - mu_b
            norm = torch.norm(diff, 2, dim=2)
            margin = 2 * self.delta_d * (1.0 - torch.eye(n))
            margin = margin.to(centroids.device)
            distance = torch.sum(torch.clamp(margin - norm, min=0.0) ** 2) # hinge loss
            distance /= float(n * (n - 1))
            loss += distance
        loss /= batch_size
        return loss

    def _regularization(self, centroids, size):
        batch_size = centroids.size(0)
        loss = 0.0
        for i in range(batch_size):
            n = size[i]
            # n = size
            mu = centroids[i, :n, :]
            norm = torch.norm(mu, 2, dim=1)
            loss += torch.mean(norm)
        loss /= batch_size
        return loss



# class DiscriminativeLoss(nn.Module):
#     def __init__(self, delta_d, delta_v,
#                  alpha=1.0, beta=1.0, gamma=0.001,
#                  reduction='mean'):
#         # TODO: Respect the reduction rule
#         super(DiscriminativeLoss, self).__init__()
#         self.alpha = alpha
#         self.beta = beta
#         self.gamma = gamma
#         # Set delta_d > 2 * delta_v
#         self.delta_d = delta_d
#         self.delta_v = delta_v

#     def forward(self, embedded, masks, sizes):
#         centroids = self._centroids(embedded, masks, sizes)
#         L_v = self._variance(embedded, masks, centroids)
#         L_d = self._distance(centroids, masks)
#         L_r = self._regularization(centroids, masks)
#         loss = self.alpha * L_v + self.beta * L_d + self.gamma * L_r
#         # print("***********start***************")
#         # print("embedded",embedded)
#         # print("masks",masks)
#         # print("size",size)
#         # print("centroids",centroids)
#         # print("L_v",L_v)
#         # print("L_d",L_d)
#         # print("L_r",L_r)
#         # print(loss)
#         return loss

#     def _centroids(self, embedded, masks, sizes):
#         batch_size = embedded.size(0)
#         embedding_size = embedded.size(2)
#         # print("tanomu")
#         # print(masks.shape)
#         # K = masks.size(2)
#         # print("kkkkkkkk")
#         #print(embedded.shape)
#         #print(size.shape)
#         # print(masks.size(2))
#         # x = embedded.unsqueeze(2).expand(-1, -1, K, -1)
#         # masks = masks.unsqueeze(3)
#         #print(x.shape)
#         #print(masks.shape)
#         # x = x * masks
#         # print("centroid")
#         # print(x)
#         centroids = []
#         for i in range(batch_size):
#             n = masks[i].shape[1]
#             x = embedded.unsqueeze(2).expand(-1, -1, n, -1)
#             masks[i] = masks[i].unsqueeze(2)
#             # print("mask")
#             # print(masks[i].shape)
#             # print(x.shape)
#             x = x * masks[i]
#             K = sizes
#             # print("daaaaaaaaaaaaaaaaa")
#             # print(x[i,:,:n].type)
#             # print(masks[i,:,:n].type)
#             mu = x[i,:,:n].sum(0) / masks[i].sum(0)
#             # print("mmmmmmmmmmmmmm")
#             # print(x[i,:,:n])
#             # print("mu")
#             # print(mu)
#             if K > n:
#                 # print("*************majika***************")
#                 m = int(K - n)
#                 filled = torch.zeros(m, embedding_size)
#                 filled = filled.to(embedded.device)
#                 mu = torch.cat([mu, filled], dim=0)
#             centroids.append(mu)
#         # print("***************majika*******")
#         # print(centroids)
#         centroids = torch.stack(centroids)
#         return centroids

#     def _variance(self, embedded, masks, centroids):
#         batch_size = embedded.size(0)
#         num_points = embedded.size(1)
#         embedding_size = embedded.size(2)
#         # K = masks.size(2)
#         # Convert input into the same size
#         # mu = centroids.unsqueeze(1).expand(-1, num_points, -1, -1)
#         # x = embedded.unsqueeze(2).expand(-1, -1, K, -1)
#         # Calculate intra pull force
#         # var = torch.norm(x - mu, 2, dim=3) #L2Nolm
#         # var = torch.clamp(var - self.delta_v, min=0.0) ** 2
#         #print("sizesize")
#         #print(var.shape)
#         # var = var * masks
#         loss = 0.0
#         for i in range(batch_size):
#             # n = size
#             n = masks[i].shape[1]
#             mu = centroids.unsqueeze(1).expand(-1, num_points, -1, -1)
#             x = embedded.unsqueeze(2).expand(-1, -1, n, -1)
#             # print("mu")
#             # print(centroids.shape)
#             # print(mu.shape)
#             # print(x.shape)
#             var = torch.norm(x - mu, 2, dim=3) #L2Nolm
#             var = torch.clamp(var - self.delta_v, min=0.0) ** 2
#             masks[i] = masks[i].transpose(2,1)
#             masks[i] = masks[i].transpose(1,0)
#             # print("maskmask")
#             # print(var.shape)
#             # print(masks[i].shape)
#             var = var * masks[i]
#             loss += torch.sum(var[i,:,:n]) / torch.sum(masks[i])
#         loss /= batch_size
#         return loss

#     def _distance(self, centroids, masks):
#         batch_size = centroids.size(0)
#         loss = 0.0
#         for i in range(batch_size):
#             # n = size
#             n = masks[i].shape[1]
#             if n <= 1: continue
#             mu = centroids[i, :n, :]
#             mu_a = mu.unsqueeze(1).expand(-1, n, -1)
#             mu_b = mu_a.permute(1, 0, 2)
#             mu_b = mu_b.transpose(1,0)
#             # print("datas")
#             # print(mu_a.shape)
#             # print(mu_b.shape)
#             diff = mu_a - mu_b
#             norm = torch.norm(diff, 2, dim=2)
#             margin = 2 * self.delta_d * (1.0 - torch.eye(n))
#             margin = margin.to(centroids.device)
#             print("sorosoro")
#             print(margin.shape)
#             print(norm.shape)
#             distance = torch.sum(torch.clamp(margin - norm, min=0.0) ** 2) # hinge loss
#             distance /= float(n * (n - 1))
#             loss += distance
#         loss /= batch_size
#         return loss

#     def _regularization(self, centroids, masks):
#         batch_size = centroids.size(0)
#         loss = 0.0
#         for i in range(batch_size):
#             # n = size
#             n = masks[i].shape[1]
#             mu = centroids[i, :n, :]
#             norm = torch.norm(mu, 2, dim=1)
#             loss += torch.mean(norm)
#         loss /= batch_size
#         return loss



# class DiscriminativeLoss(nn.Module):
#     def __init__(self, delta_d, delta_v,
#                  alpha=1.0, beta=1.0, gamma=0.001,
#                  reduction='mean'):
#         # TODO: Respect the reduction rule
#         super(DiscriminativeLoss, self).__init__()
#         self.alpha = alpha
#         self.beta = beta
#         self.gamma = gamma
#         # Set delta_d > 2 * delta_v
#         self.delta_d = delta_d
#         self.delta_v = delta_v

#     def forward(self, embedded, masks, sizes):
#         centroids = self._centroids(embedded, masks, sizes)
#         L_v = self._variance(embedded, masks, centroids, sizes)
#         L_d = self._distance(centroids, sizes)
#         L_r = self._regularization(centroids, sizes)
#         loss = self.alpha * L_v + self.beta * L_d + self.gamma * L_r
#         # print("***********start***************")
#         # print("embedded",embedded)
#         # print("masks",masks)
#         # print("size",size)
#         # print("centroids",centroids)
#         # print("L_v",L_v)
#         # print("L_d",L_d)
#         # print("L_r",L_r)
#         # print(loss)
#         return loss

#     def _centroids(self, embedded, masks, sizes):
#         batch_size = embedded.size(0)
#         embedding_size = embedded.size(2)
#         K = masks.size(2)
#         # print("kkkkkkkk")
#         #print(embedded.shape)
#         #print(size.shape)
#         # print(masks.size(2))
#         x = embedded.unsqueeze(2).expand(-1, -1, K, -1)
#         masks = masks.unsqueeze(3)
#         #print(x.shape)
#         #print(masks.shape)
#         x = x * masks
#         # print("mask")
#         # print(masks.shape)
#         # print(x.shape)
#         # print("centroid")
#         # print(x)
#         centroids = []
#         for i in range(batch_size):
#             # n = np.shape(masks[i][1])
#             n = sizes[i]
#             # print("daaaaaaaaaaaaaaaaa")
#             # print(x[i,:,:n].type)
#             # print(masks[i,:,:n].type)
#             mu = x[i,:,:n].sum(0) / masks[i,:,:n].sum(0)
#             # print("mmmmmmmmmmmmmm")
#             # print(x[i,:,:n])
#             # print("mu")
#             # print(mu)
#             if K > n:
#                 # print("*************majika***************")
#                 m = int(K - n)
#                 filled = torch.zeros(m, embedding_size)
#                 filled = filled.to(embedded.device)
#                 mu = torch.cat([mu, filled], dim=0)
#             centroids.append(mu)
#         # print("***************majika*******")
#         # print(centroids)
#         centroids = torch.stack(centroids)
#         return centroids

#     def _variance(self, embedded, masks, centroids, sizes):
#         batch_size = embedded.size(0)
#         num_points = embedded.size(1)
#         embedding_size = embedded.size(2)
#         K = masks.size(2)
#         # Convert input into the same size
#         mu = centroids.unsqueeze(1).expand(-1, num_points, -1, -1)
#         x = embedded.unsqueeze(2).expand(-1, -1, K, -1)
#         # Calculate intra pull force
#         var = torch.norm(x - mu, 2, dim=3) #L2Nolm
#         var = torch.clamp(var - self.delta_v, min=0.0) ** 2
#         #print("sizesize")
#         #print(var.shape)
#         var = var * masks
#         loss = 0.0
#         # print("mu")
#         # print(centroids.shape)
#         # print(mu.shape)
#         # print(x.shape)
#         var = torch.norm(x - mu, 2, dim=3) #L2Nolm
#         var = torch.clamp(var - self.delta_v, min=0.0) ** 2
#         # print("maskmask")
#         # print(var.shape)
#         # print(masks.shape)
#         for i in range(batch_size):
#             n = sizes[i]
#             # n = np.shape(masks[i][1])
#             loss += torch.sum(var[i,:,:n]) / torch.sum(masks[i,:,:n])
#         loss /= batch_size
#         return loss

#     def _distance(self, centroids, sizes):
#         batch_size = centroids.size(0)
#         loss = 0.0
#         for i in range(batch_size):
#             n = sizes[i]
#             # n = np.shape(masks[i][1])
#             if n <= 1: continue
#             mu = centroids[i, :n, :]
#             mu_a = mu.unsqueeze(1).expand(-1, n, -1)
#             mu_b = mu_a.permute(1, 0, 2)
#             # print("datas")
#             # print(mu_a.shape)
#             # print(mu_b.shape)
#             diff = mu_a - mu_b
#             norm = torch.norm(diff, 2, dim=2)
#             margin = 2 * self.delta_d * (1.0 - torch.eye(n))
#             margin = margin.to(centroids.device)
#             distance = torch.sum(torch.clamp(margin - norm, min=0.0) ** 2) # hinge loss
#             distance /= float(n * (n - 1))
#             loss += distance
#         loss /= batch_size
#         return loss

#     def _regularization(self, centroids, sizes):
#         batch_size = centroids.size(0)
#         loss = 0.0
#         for i in range(batch_size):
#             n = sizes[i]
#             # n = np.shape(masks[i][1])
#             mu = centroids[i, :n, :]
#             norm = torch.norm(mu, 2, dim=1)
#             loss += torch.mean(norm)
#         loss /= batch_size
#         return loss
