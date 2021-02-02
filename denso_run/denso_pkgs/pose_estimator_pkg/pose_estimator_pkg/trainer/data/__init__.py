import torch.utils.data
from torch.utils.data.dataset import Subset
from data.base_dataset import collate_fn
import sys

def TrainValDataset(opt):
    if opt.dataset_mode == "pose_estimation":
        from data.pose_estimate_data import PoseData
        dataset = PoseData(opt)
        n_samples = len(dataset)
        train_size = int(n_samples * 0.95)

        subset1_indices = list(range(0, train_size))
        subset2_indices = list(range(train_size, n_samples))

        subset1 = Subset(dataset, subset1_indices)
        subset2 = Subset(dataset, subset2_indices)
    else:
        print("Error!! ")
        sys.exit(1)
    return subset1, subset2


class TrainDataLoader:
    def __init__(self, dataset, opt):
        self.opt = opt
        self.dataset= dataset
        self.batch_size = opt.batch_size * opt.gpu_num
        self.dataloader = torch.utils.data.DataLoader(
                self.dataset,
                batch_size=self.batch_size,
                shuffle=True,
                num_workers=int(opt.num_threads),
                collate_fn=collate_fn)

    def __len__(self):
        return min(len(self.dataset), self.opt.max_dataset_size)


    def __iter__(self):
        for i, data in enumerate(self.dataloader):
            if i * self.opt.batch_size >= self.opt.max_dataset_size:
                break
            yield data

class ValDataLoader:
    def __init__(self, dataset, opt):
        self.opt = opt
        self.dataset= dataset
        self.batch_size = opt.batch_size * opt.gpu_num
        self.dataloader = torch.utils.data.DataLoader(
                self.dataset,
                self.batch_size,
                shuffle=False,
                num_workers=int(opt.num_threads),
                collate_fn=collate_fn)

    def __len__(self):
        return min(len(self.dataset), self.opt.max_dataset_size)

    def __iter__(self):
        for i, data in enumerate(self.dataloader):
            if i * self.opt.batch_size >= self.opt.max_dataset_size:
                break
            yield data
