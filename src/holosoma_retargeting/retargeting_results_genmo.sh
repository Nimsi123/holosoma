source ~/.holosoma_deps/miniconda3/bin/activate hssim

# # 30k iterations works well
# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/65ces61EZhM_182173_182778_0_1_2_8_0_original.npz


# to be tested

# retargeting looks fine
# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_1_79C7Sxy0xro_102205_102710_1_9_0_4_0_genmo_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_1_79C7Sxy0xro_236656_237221_1_0_1_2_0_genmo_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_2_Ft30daV_EVY_593041_593801_2_7_2_4_0_genmo_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_2_kWggJar3pIY_183256_183816_1_6_1_5_0_genmo_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_2_RQH8irMiTLY_296692_297297_1_10_0_10_0_genmo_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_2_wCo9atjANTo_219320_219990_3_9_1_7_0_genmo_original.npz

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz /bluesclues-data/home/pingpong-nima/holosoma/src/holosoma_retargeting/demo_results_parallel/g1/robot_only/tt4d_mixtape_genmo/batch_clipped_long_2_wCo9atjANTo_403565_404205_1_2_2_0_0_genmo_original.npz

# test files

python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz /bluesclues-data/home/pingpong-nima/robot_table_tennis/src/humanoid_mixtape/test_tracking/test_data/batch_clipped_long_1_65ces61EZhM_182173_182778_0_1_2_8_0_genmo/holosoma_retarget.npz