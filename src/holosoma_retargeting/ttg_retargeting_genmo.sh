source ~/.holosoma_deps/miniconda3/bin/activate hssim
# python examples/robot_retarget.py --data_path demo_data/tt4d_mixtape_genmo --task-type robot_only --task-name 65ces61EZhM_182173_182778_0_1_2_8_0 --data_format tt4d --save_dir demo_results/g1/robot_only/tt4d_mixtape_genmo --retargeter.debug --retargeter.visualize

python examples/robot_retarget.py --data_path demo_data/tt4d_mixtape_genmo --task-type robot_only --task-name batch_clipped_long_1_79C7Sxy0xro_102205_102710_1_9_0_4_0_genmo --data_format tt4d --save_dir demo_results/g1/robot_only/tt4d_mixtape_genmo --retargeter.debug --retargeter.visualize

