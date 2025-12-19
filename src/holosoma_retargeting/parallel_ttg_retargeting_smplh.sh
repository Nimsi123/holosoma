source ~/.holosoma_deps/miniconda3/bin/activate hssim

# rm -r demo_results_parallel/g1/robot_only/tt4d_mixtape
# python examples/parallel_robot_retarget.py --data-dir demo_data/tt4d_mixtape --task-type robot_only --data_format tt4d --save_dir demo_results_parallel/g1/robot_only/tt4d_mixtape --task-config.object-name ground

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape/-7lbDSIKUak_212395_212765_0_1_0_2_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape/--7lbDSIKUak_214445_214810_0_4_0_6_original.npz

# good example of the feet needing to be fixed
# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape/87TgS5kIP-M_148000_148690_0_7_0_7_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape/-7lbDSIKUak_232930_233555_0_1_2_2_original.npz


# # after SMPL trajectory filtering
# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /home/nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape/4deOUpU2sis_269666_270406_1_7_2_8_original.npz





# we don't even need this script. we run this in beyondmimic
# python data_conversion/parallel_convert_data_format_mj.py \
#     --input_dir demo_results_parallel/g1/robot_only/tt4d_mixtape \
#     --output_dir converted_res_parallel/robot_only/tt4d_mixtape \
#     --output_fps 30 \
#     --data_format tt4d \
#     --object_name ground \
#     --max_workers 8 \
#     --file_pattern "*.npz"