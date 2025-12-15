source ~/.holosoma_deps/miniconda3/bin/activate hssim

# python examples/parallel_robot_retarget.py --data-dir demo_data/motive_gameplay --task-type robot_only --data_format motive_gameplay --save_dir demo_results_parallel/g1/robot_only/motive_gameplay --task-config.object-name ground

# Decent. I think this is Milo. 
# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz "/home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/motive_gameplay/Take 2025-09-29 05.07.06 PM_part003_15_original.npz"

# # Nice and pretty long! I think this is Milo. 
# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz "/home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/motive_gameplay/Take 2025-09-29 05.33.02 PM_part000_1_original.npz"



# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz "/home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/motive_gameplay/Take_2025-09-30_03.38.11_PM_part000_9_3_original.npz"


# debug
python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz "/home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/motive_gameplay/Take_2025-09-30_03.38.11_PM_part000_9_4_original.npz"
