def create_model(out):
    from .pose_estimate import EstimatorModel
    model = EstimatorModel(out)
    return model
