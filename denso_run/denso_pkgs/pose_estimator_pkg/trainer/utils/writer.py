import os
import time

try:
    from tensorboardX import SummaryWriter
except ImportError as error:
    print("tensorboardX not installed, visualizing wont be available")
    SummaryWriter = None


class Writer:
    def __init__(self, opt):
        self.opt = opt
        self.name = self.opt.name
        self.dataset_model = self.opt.dataset_model
        self.save_dir = os.path.join(opt.checkpoints_dir, self.name, self.dataset_model)
        self.log_name = os.path.join(self.save_dir, "loss_log.txt")
        self.testacc_log = os.path.join(self.save_dir, "testacc_log.txt")
        self.start_logs()
        self.nexampels = 0
        self.ncorrect = 0

        if opt.is_train and not opt.no_vis and SummaryWriter is not None:
            self.display = SummaryWriter(comment=self.name + "_" + self.dataset_model)
        else:
            self.display = None


    def start_logs(self):
        if self.opt.is_train:
            with open(self.log_name, "a") as log_file:
                now = time.strftime("%c")
                log_file.write('============Training Loss (%s)===========\n' % now)
        else:
            with open(self.testacc_log, "a") as log_file:
                now = time.strftime("%c")
                log_file.write('===========Test Acc (%s)================\n' %now)

    def print_current_losses(self, phase, epoch, i, losses, t, t_data):
        """ prints train loss on terminal / file """
        message = '(phase: %s, epoch: %d, iters: %d, time: %.3f, data:%.3f) loss: %.3f ' %(phase, epoch, i, t, t_data, losses)
        print(message)
        with open(self.log_name, "a") as log_file:
            log_file.write('%s\n' % message)

    def plot_loss(self, loss, epoch, i, n):
        iters = i + (epoch - 1) * n
        if self.display:
            self.display.add_scalar("data/current_train_loss", loss, iters)

    def plot_model_wts(self, model, epoch):
        if self.opt.is_train and self.display:
            for name, param in model.net.named_parameters():
                self.display.add_histogram(name, param.clone().cpu().data.numpy(), epoch)


    def plot_loss(self, epoch, loss, val_loss):
        if self.display:
            self.display.add_scalar("data/train_loss", loss, epoch)
            self.display.add_scalar("data/val_loss", val_loss, epoch)


    def close(self):
        if self.display is not None:
            self.display.close()

