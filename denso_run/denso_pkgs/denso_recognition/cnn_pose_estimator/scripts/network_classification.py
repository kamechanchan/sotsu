# -*- coding:utf-8 -*-

import chainer
import chainer.functions as F
import chainer.links as L
from chainer import report

output_pos_num = 3
output_ori_num = 3
output_class = 2


class CNN(chainer.Chain):

    def __init__(self):
        super(CNN, self).__init__(
            conv1=L.ConvolutionND(
    ndim=3,
     in_channels=1,
     out_channels=5,
     ksize=3,
     pad=1,
     nobias=True),
            conv2=L.ConvolutionND(
    ndim=3,
     in_channels=5,
     out_channels=10,
     ksize=3,
     pad=1,
     nobias=True),
            conv3=L.ConvolutionND(
    ndim=3,
     in_channels=10,
     out_channels=5,
     ksize=3,
     pad=1,
     nobias=True),

            l_pos1=L.Linear(None, 1000),
            l_pos2=L.Linear(1000, 500),
# l_pos3 = L.Linear(500, 200),
            l_pos=L.Linear(500, output_pos_num),

            l_ori1=L.Linear(None, 1000),
            l_ori2=L.Linear(1000, 500),
            l_ori3=L.Linear(500, 100),
            l_ori=L.Linear(100, output_ori_num),

            l_class1=L.Linear(None, 800),
            l_class2=L.Linear(800, 100),
            l_class=L.Linear(100, output_class),
        )

    def __call__(self, x, t):
        h_r, h_c = self.fwd(x)
        loss_r = F.mean_squared_error(h_r, t)
        loss_c = F.softmax_cross_entropy(h_c, t)
        report({'loss': loss_r}, self)
        report({'accuracy': loss_c}, self)
        return loss_r, loss_c

    def fwd(self, x):
        h = self.conv1(x)
        h = F.max_pooling_nd(h, 3, stride=2)
        h = self.conv2(h)
        h = F.max_pooling_nd(h, 3, stride=2)
        h = self.conv3(h)
        h = F.max_pooling_nd(h, 3, stride=2)

        h1 = F.relu(self.l_pos1(h))
        h1 = F.relu(self.l_pos2(h1))
        h1 = F.dropout(h1, ratio=0.5)
        h1 = F.sigmoid(self.l_pos(h1))

        h2 = F.relu(self.l_ori1(h))
        h2 = F.dropout(h2, ratio=0.5)
        h2 = F.relu(self.l_ori2(h2))
        h2 = F.relu(self.l_ori3(h2))
        h2 = F.tanh(self.l_ori(h2))

        hc = F.relu(self.l_class1(h))
        hc = F.dropout(h2, ratio=0.5)
        hc = F.relu(self.l_class2(hc))
        hc = F.softmax(self.l_class(hc))

        y_r = F.concat((h1, h2), axis=1)
        y_c = hc

        return y_r, y_c

    def pre(self, x):
        h = self.conv1(x)
        h = F.max_pooling_nd(h, 3, stride=2)
        h = self.conv2(h)
        h = F.max_pooling_nd(h, 3, stride=2)
        h = self.conv3(h)
        h = F.max_pooling_nd(h, 3, stride=2)

        h1 = F.relu(self.l_pos1(h))
        h1 = F.relu(self.l_pos2(h1))
        h1 = F.sigmoid(self.l_pos(h1))

        h2 = F.relu(self.l_ori1(h))
        h2 = F.relu(self.l_ori2(h2))
        h2 = F.relu(self.l_ori3(h2))
        h2 = F.tanh(self.l_ori(h2))

        hc = F.relu(self.l_class1(h))
        hc = F.dropout(h2, ratio=0.5)
        hc = F.relu(self.l_class2(hc))
        hc = F.tanh(self.l_class(hc))

        y_r = F.concat((h1, h2, axis=1)
        y_c=hc

        return y_r, y_c

    def predict(self, x):
        h_r, h_c=self.pre(x)
        return h_r, h_c
