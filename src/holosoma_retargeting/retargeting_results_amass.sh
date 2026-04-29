source ~/.holosoma_deps/miniconda3/bin/activate hssim

# after SMPL trajectory filtering
python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz "/home/nima/motion_retargeting_prior/data/1_motion_tracking/amass/ACCAD_Female1General_c3d_A14_-_stand_to_skip_poses/ACCAD_Female1General_c3d_A14_-_stand_to_skip_poses.npz"




