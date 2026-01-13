source ~/.holosoma_deps/miniconda3/bin/activate hssim

rm -r demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo
python examples/parallel_robot_retarget.py --data-dir demo_data/tt4d_mixtape_genmo --task-type robot_only --data_format tt4d --save_dir demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo --task-config.object-name ground

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/65ces61EZhM_182173_182778_0_1_2_8_0_original.npz