source ~/.holosoma_deps/miniconda3/bin/activate hssim
python examples/robot_retarget.py --data_path demo_data/motive_gameplay --task-type robot_only --task-name Take_2025-09-30_03.38.11_PM_part000_9_4 --data_format motive_gameplay --save_dir demo_results/g1/robot_only/motive_gameplay --retargeter.debug --retargeter.visualize

# python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
#     --qpos_npz demo_data/motive_gameplay/Take_2025-09-30_03.38.11_PM_part000_9_3.pt

# python data_conversion/convert_data_format_mj.py --input_file demo_results/g1/robot_only/motive_gameplay/motive_gameplay_ttg_test_1.npz --output_fps 30 --output_name converted_res/robot_only/motive_gameplay_ttg_test_1.npz --data_format motive_gameplay --object_name "ground" --once --no_viewer