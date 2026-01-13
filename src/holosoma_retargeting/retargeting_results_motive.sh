source ~/.holosoma_deps/miniconda3/bin/activate hssim

# after SMPL trajectory filtering
python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz "/home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/motive_gameplay/Take 2025-09-29 05.07.06 PM_part001_11_original.npz"


