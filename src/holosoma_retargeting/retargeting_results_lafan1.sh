source ~/.holosoma_deps/miniconda3/bin/activate hssim

# after SMPL trajectory filtering
python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz "/home/nima/motion_retargeting_prior/data/1_motion_tracking/lafan1/aiming1_subject1/aiming1_subject1.npz"


