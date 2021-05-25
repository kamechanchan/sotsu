from base_options import BaseOptions


class TestOptions(BaseOptions):
    def initialize(self):
        BaseOptions.initialize(self)
        self.parser.add_argument("--results_dir", type=str, default="../../weights")
        self.parser.add_argument("--phase", type=str, default="test", help="train, val, test")
        self.parser.add_argument("--which_epoch", type=str, default="latest")
        self.parser.add_argument('--is_train', type=bool, default=False)
        self.is_train=False



