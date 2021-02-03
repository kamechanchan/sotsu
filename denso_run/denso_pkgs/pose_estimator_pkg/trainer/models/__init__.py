def create_model(opt):
    from .pose_estimate import EstimatorModel
    model = EstimatorModel(opt)
    return model
