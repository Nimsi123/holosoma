# python examples/robot_retarget.py --data_path demo_data/OMOMO_new --task-type robot_only --task-name sub3_largebox_003 --data_format smplh --retargeter.debug --retargeter.visualize

source ~/.holosoma_deps/miniconda3/bin/activate hssim
# python examples/robot_retarget.py --data_path demo_data/tt4d --task-type robot_only --task-name smplh_ttg_test_1 --data_format tt4d --save_dir ~/holosoma/src/holosoma_retargeting/demo_results/g1/robot_only/tt4d/smplh_ttg_test_1.npzdemo_results/g1/robot_only/tt4d --retargeter.debug --retargeter.visualize

python viser_player.py --robot_urdf models/g1/g1_29dof.urdf \
    --qpos_npz demo_results/g1/robot_only/tt4d/smplh_ttg_test_1.npz

python data_conversion/convert_data_format_mj.py --input_file demo_results/g1/robot_only/tt4d/smplh_ttg_test_1.npz --output_fps 30 --output_name converted_res/robot_only/smplh_ttg_test_1.npz --data_format smplh --object_name "ground" --once --no_viewer
