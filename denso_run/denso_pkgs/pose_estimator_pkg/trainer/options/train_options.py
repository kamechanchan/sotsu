from .base_options import BaseOptions

class TrainOptions(BaseOptions):
    def initialize(self):
        print('train_option')
        BaseOptions.initialize(self)
        self.parser.add_argument('--dataroot', required=True)
        self.parser.add_argument('--print_freq', type=int, default=10, help='frequency of showing training results on console')
        self.parser.add_argument('--save_latest_freq', type=int, default=250, help='frequency of saving the latest results')
        self.parser.add_argument('--save_epoch_freq', type=int, default=1, help='frequency of saving checkpoints at the end of epochs')
        self.parser.add_argument('--run_test_freq', type=int, default=1, help='frequency of running test in training script')
        self.parser.add_argument('--continue_train', action="store_true", help='continue training: load the latest model')
        self.parser.add_argument('--epoch_count', type=int, default= 1, help='starting epoch count, we save the model by <epoch_count>, <epoch_count>+<save_latest_freq>, ...')
        self.parser.add_argument('--phase', type=str, default="trainval", help='train, val or trainval')
        self.parser.add_argument('--which_epoch', type=str, default="latest", help='which epoch to load? set to latest to use latest cached model')
        self.parser.add_argument('--niter', type=int, default=100, help='of iter at starting learning rate')
        self.parser.add_argument('--niter_decay', type=int, default=500, help='of iter to lineary decay learing rate to zero')
        self.parser.add_argument('--beta1', type=float, default=0.9, help='momentum term of adam')
        
        self.parser.add_argument('--lr_policy', type=str, default="lambda", help='learning rate policy: lambda|step|plateau')
        self.parser.add_argument('--lr_decay_iters', type=int, default=50, help='multiply by a gamma every lr_decay_iters iterations')

        # for ternsorboard visualization
        self.parser.add_argument('--no_vis', action='store_true', help='will not use tensorboard')
        self.parser.add_argument('--verbose_plot', action='store_true', help='plots network weights, etc.')
        
        #self.is_train = True


