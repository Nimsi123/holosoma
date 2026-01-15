"""
Parallel retargeting script for test_data folder structure.
Each subdirectory contains a motion_genmo.pt file and results are saved in the same subdirectory.
"""

from __future__ import annotations

import multiprocessing as mp
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path
from typing import Literal

import tyro

# Add holosoma src to path (same as parallel_robot_retarget.py)
src_root = Path(__file__).resolve().parents[2]
if str(src_root) not in sys.path:
    sys.path.insert(0, str(src_root))

from holosoma_retargeting.config_types.data_type import MotionDataConfig  # noqa: E402
from holosoma_retargeting.config_types.retargeting import RetargeterConfig  # noqa: E402
from holosoma_retargeting.config_types.robot import RobotConfig  # noqa: E402
from holosoma_retargeting.config_types.task import TaskConfig  # noqa: E402

# Import reusable functions from robot_retarget.py
from holosoma_retargeting.examples.robot_retarget import (  # noqa: E402
    build_retargeter_kwargs_from_config,
    create_task_constants,
    initialize_robot_pose,
    load_motion_data,
    setup_object_data,
)
from holosoma_retargeting.src.interaction_mesh_retargeter import (  # noqa: E402
    InteractionMeshRetargeter,
)
from holosoma_retargeting.src.utils import (  # noqa: E402
    extract_foot_sticking_sequence_velocity,
    preprocess_motion_data,
)


@dataclass
class TestDataRetargetingConfig:
    """Configuration for test_data retargeting."""

    # Root directory containing subdirectories with motion_genmo.pt files
    data_dir: Path = Path(
        "/bluesclues-data/home/pingpong-nima/robot_table_tennis/src/humanoid_mixtape/test_tracking/test_data"
    )

    # Motion file name to look for in each subdirectory
    motion_filename: str = "motion_genmo.pt"

    # Output filename (will be saved in the same directory as input)
    output_filename: str = "holosoma_retarget.npz"

    # Robot type
    robot: Literal["g1", "h1"] = "g1"

    # Data format
    data_format: Literal["tt4d"] = "tt4d"

    # Task type
    task_type: Literal["robot_only"] = "robot_only"

    # Maximum number of parallel workers (None = use all CPUs)
    max_workers: int | None = None

    # Sub-configs
    robot_config: RobotConfig = field(default_factory=lambda: RobotConfig(robot_type="g1"))
    motion_data_config: MotionDataConfig = field(
        default_factory=lambda: MotionDataConfig(data_format="tt4d", robot_type="g1")
    )
    task_config: TaskConfig = field(default_factory=lambda: TaskConfig(object_name="ground"))
    retargeter: RetargeterConfig = field(default_factory=RetargeterConfig)


def find_motion_subdirs(data_dir: Path, motion_filename: str) -> list[tuple[Path, Path]]:
    """Find all subdirectories containing the specified motion file.

    Args:
        data_dir: Root directory to search
        motion_filename: Name of motion file to look for

    Returns:
        List of tuples (motion_file_path, subdirectory_path)
    """
    data_dir = Path(data_dir)
    results = []

    for subdir in sorted(data_dir.iterdir()):
        if subdir.is_dir():
            motion_file = subdir / motion_filename
            if motion_file.exists():
                results.append((motion_file, subdir))

    return results


def process_single_motion(args: tuple) -> None:
    """Process a single motion file and save results in the same directory.

    Args:
        args: Tuple containing (motion_file, save_dir, task_type, data_format,
              robot_config, motion_data_config, task_config, retargeter_config, output_filename)
    """
    (
        motion_file,
        save_dir,
        task_type,
        data_format,
        robot_config,
        motion_data_config,
        task_config,
        retargeter_config,
        output_filename,
    ) = args

    motion_file = Path(motion_file)
    save_dir = Path(save_dir)
    task_name = motion_file.stem  # e.g., "motion_genmo"
    output_path = save_dir / output_filename

    print(f"Processing: {motion_file.parent.name}/{motion_file.name}")

    # Skip if output already exists
    if output_path.exists():
        print(f"  Skipping (output exists): {output_path}")
        return

    # Create task constants
    constants = create_task_constants(robot_config, motion_data_config, task_config, task_type)

    # Load motion data
    human_joints, object_poses, smpl_scale = load_motion_data(
        task_type, data_format, motion_file.parent, task_name, constants, motion_data_config
    )

    # Get toe names from motion data config
    toe_names = motion_data_config.toe_names

    # Setup object data
    object_local_pts, object_local_pts_demo, object_urdf_path = setup_object_data(
        task_type,
        constants,
        task_config.object_dir,
        smpl_scale,
        task_config,
        augmentation=False,
    )

    # Create retargeter
    retargeter_kwargs = build_retargeter_kwargs_from_config(
        retargeter_config, constants, object_urdf_path, task_type
    )
    retargeter = InteractionMeshRetargeter(**retargeter_kwargs)

    # Preprocess motion data
    human_joints = preprocess_motion_data(human_joints, retargeter, toe_names, smpl_scale)

    # Extract foot sticking sequences
    foot_sticking_sequences = extract_foot_sticking_sequence_velocity(
        human_joints, retargeter.demo_joints, toe_names
    )

    # Initialize robot pose
    q_init, q_nominal, object_poses_augmented, human_joints, object_poses = initialize_robot_pose(
        task_type,
        data_format,
        human_joints,
        object_poses,
        constants,
        retargeter,
        task_config,
        augmentation=False,
        save_dir=save_dir,
        task_name=task_name,
    )

    # Retarget motion
    retargeted_motions, _, _, _ = retargeter.retarget_motion(
        human_joint_motions=human_joints,
        object_poses=object_poses,
        object_poses_augmented=object_poses_augmented,
        object_points_local_demo=object_local_pts_demo,
        object_points_local=object_local_pts,
        foot_sticking_sequences=foot_sticking_sequences,
        q_a_init=q_init,
        q_nominal_list=q_nominal,
        original=True,
        dest_res_path=str(output_path),
    )

    print(f"  Saved: {output_path}")


def main(cfg: TestDataRetargetingConfig) -> None:
    """Main parallel retargeting pipeline for test_data folder structure."""

    # Ensure configs match top-level selections
    if cfg.robot_config.robot_type != cfg.robot:
        cfg.robot_config = RobotConfig(robot_type=cfg.robot)

    if cfg.motion_data_config.robot_type != cfg.robot or cfg.motion_data_config.data_format != cfg.data_format:
        cfg.motion_data_config = MotionDataConfig(data_format=cfg.data_format, robot_type=cfg.robot)

    # Find all motion files
    motion_items = find_motion_subdirs(cfg.data_dir, cfg.motion_filename)
    print(f"Found {len(motion_items)} subdirectories with {cfg.motion_filename}")

    if not motion_items:
        print("No motion files found. Exiting.")
        return

    # Prepare arguments for parallel processing
    process_args = [
        (
            str(motion_file),
            str(subdir),
            cfg.task_type,
            cfg.data_format,
            cfg.robot_config,
            cfg.motion_data_config,
            cfg.task_config,
            cfg.retargeter,
            cfg.output_filename,
        )
        for motion_file, subdir in motion_items
    ]

    # Set up parallel processing
    max_workers = cfg.max_workers or mp.cpu_count()
    print(f"Using {max_workers} parallel workers")

    start_time = time.time()
    successful = 0
    failed = 0

    # Process files in parallel
    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        future_to_file = {
            executor.submit(process_single_motion, arg): arg[0] for arg in process_args
        }

        for future in as_completed(future_to_file):
            file_path = future_to_file[future]
            try:
                future.result()
                print(f"Completed: {Path(file_path).parent.name}")
                successful += 1
            except Exception as e:
                print(f"Failed {Path(file_path).parent.name}: {e}")
                import traceback

                traceback.print_exc()
                failed += 1

    end_time = time.time()

    print("\n=== Processing Summary ===")
    print(f"Data directory: {cfg.data_dir}")
    print(f"Total subdirectories: {len(motion_items)}")
    print(f"Successful: {successful}")
    print(f"Failed: {failed}")
    print(f"Total time: {end_time - start_time:.2f} seconds")
    if len(motion_items) > 0:
        print(f"Average time per file: {(end_time - start_time) / len(motion_items):.2f} seconds")


if __name__ == "__main__":
    cfg = tyro.cli(TestDataRetargetingConfig)
    main(cfg)

