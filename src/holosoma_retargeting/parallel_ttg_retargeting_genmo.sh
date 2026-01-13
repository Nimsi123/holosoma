source ~/.holosoma_deps/miniconda3/bin/activate hssim

# rm -r demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo
# python examples/parallel_robot_retarget.py --data-dir demo_data/tt4d_mixtape_genmo --task-type robot_only --data_format tt4d --save_dir demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo --task-config.object-name ground

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/6b8DKaBOGdM_157776_158301_1_3_0_1_0_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/7NiBrGCOMsY_101194_101734_1_1_0_2_0_original.npz

python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/65ces61EZhM_182173_182778_0_1_2_8_0_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/79C7Sxy0xro_236656_237221_1_0_1_2_0_original.npz


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