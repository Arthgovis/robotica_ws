# amcl.yaml — configuracao do AMCL para localizacao com mapa pre-construido

amcl:
  ros__parameters:

    # Frames
    global_frame_id:      map
    odom_frame_id:        odom
    base_frame_id:        base_footprint
    scan_topic:           scan

    # Numero de particulas
    min_particles: 500
    max_particles: 2000

    # Modelo do laser (likelihood_field e mais eficiente para 2D)
    laser_model_type:           likelihood_field
    laser_max_range:            12.0
    laser_min_range:            0.3
    max_beams:                  60
    laser_likelihood_max_dist:  2.0

    # Modelo de movimento diferencial
    robot_model_type: nav2_amcl::DifferentialMotionModel
    alpha1: 0.2   # ruido rot->rot
    alpha2: 0.2   # ruido trans->rot
    alpha3: 0.2   # ruido trans->trans
    alpha4: 0.2   # ruido rot->trans

    # Atualizacao de particulas
    update_min_d:   0.25   # distancia minima para atualizar [m]
    update_min_a:   0.2    # angulo minimo para atualizar [rad]
    resample_interval: 1

    # Pose inicial (0,0,0 com alta incerteza)
    set_initial_pose: true
    initial_pose_x:   0.0
    initial_pose_y:   0.0
    initial_pose_a:   0.0
    initial_cov_xx:   0.25
    initial_cov_yy:   0.25
    initial_cov_aa:   0.07

    transform_tolerance: 0.5
    tf_broadcast: true