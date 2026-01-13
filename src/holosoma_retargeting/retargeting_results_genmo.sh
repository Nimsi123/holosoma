source ~/.holosoma_deps/miniconda3/bin/activate hssim

# 30k iterations works well
python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/65ces61EZhM_182173_182778_0_1_2_8_0_original.npz