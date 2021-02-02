#! bin/bash

rosservice call /gazebo/set_physics_properties "
time_step: 0.001
max_upadate_rate: 1000.0
gravity:
    x: 0.0
    y: 0.0
    z: 0.0
ode_config:
    auto_disable_bodies: False
    sor_pgs_precon_iters: 0
    sor_pgs_w: 1.3
    sor_pgs_rms_error_tol: 0.0
    contact_surface_layer: 0.001
    contact_max_correction_vel: 100.0
    cfm: 0.0
    erp: 0.2
    max_contacts:20"

rosservice call /gazebo/delete_model "model_name: 'ground_plane'"
