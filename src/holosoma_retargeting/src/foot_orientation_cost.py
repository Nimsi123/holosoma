"""Cost function to penalize feet not pointing towards the ground.

This cost ALWAYS applies (not just when foot sticking is active).
It penalizes when the foot's local Z-axis deviates from being parallel to the world Z-axis,
ensuring the sole faces downward.
"""

from __future__ import annotations

from typing import Callable

import cvxpy as cp
import mujoco
import numpy as np


# Weight for the orientation cost
W_FOOT_ORIENTATION = 10.0


def make_feet_pointing_down_cost(
    *,
    robot_model,
    robot_data,
    foot_links: dict[str, str],
    q_a_indices: np.ndarray,
    dqa: cp.Variable,
    weight: float = W_FOOT_ORIENTATION,
    calc_rotation_jacobian: Callable[[int], np.ndarray],
) -> cp.Expression:
    """
    Cost that penalizes feet orientation deviating from pointing down.
    
    The foot's local Z-axis should be aligned with the world Z-axis (sole parallel to ground).
    This is achieved by penalizing the x and y components of the foot's local Z-axis
    when expressed in the world frame.
    
    For a foot with sole facing down:
    - foot_z_world should be [0, 0, 1] (or [0, 0, -1] depending on convention)
    - We penalize foot_z_world[0]^2 + foot_z_world[1]^2 (deviation from vertical)
    
    Parameters
    ----------
    robot_model, robot_data:
        MuJoCo model/data (must already have mj_forward called with current q).
    foot_links:
        Mapping {key -> mujoco_body_name} for the feet.
    q_a_indices:
        Actuated indices slice into q.
    dqa:
        CVXPY decision variable for delta-actuated.
    weight:
        Scalar weight for the orientation penalty.
    calc_rotation_jacobian:
        Callable that returns (3 x nq) rotational Jacobian for a body.
        Signature: Jr = calc_rotation_jacobian(body_id) -> (3, nq)
    
    Returns
    -------
    cp.Expression
        A CVXPY expression to add to the objective.
    """
    if weight <= 0.0:
        return 0.0

    total_cost = 0.0

    for foot_key, body_name in foot_links.items():
        # Get MuJoCo body id
        body_id = mujoco.mj_name2id(robot_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise ValueError(f"Body '{body_name}' (from foot_links['{foot_key}']) not found in MuJoCo model.")

        # Current foot orientation (rotation matrix from body to world)
        R_WB = robot_data.xmat[body_id].reshape(3, 3)
        
        # Foot's local Z-axis expressed in world frame
        # This is the third column of the rotation matrix
        foot_z_world = R_WB[:, 2]  # (3,)
        
        # Current error: we want foot_z_world to be [0, 0, ±1]
        # So the error in x and y components should be zero
        # err_xy = [foot_z_world[0], foot_z_world[1]]
        err_xy_0 = foot_z_world[:2]  # Current xy components (should be ~0)
        
        # Get rotational Jacobian for this body
        Jr = calc_rotation_jacobian(body_id)  # (3, nq)
        Jr_act = Jr[:, q_a_indices]  # (3, nq_a)
        
        # Linearize the change in foot_z_world due to rotation
        # When the body rotates by small angle δω (in world frame),
        # the new z-axis is approximately: z_new ≈ z_old + δω × z_old
        # 
        # δω = Jr @ dq, so:
        # z_new ≈ z_old + (Jr @ dq) × z_old
        #       = z_old + skew(z_old)^T @ Jr @ dq
        #       = z_old - skew(z_old) @ Jr @ dq
        #
        # For the xy components:
        # err_xy_new ≈ err_xy_0 - [skew(z_old) @ Jr @ dq]_{xy}
        
        # Skew-symmetric matrix of foot_z_world
        z = foot_z_world
        skew_z = np.array([
            [0, -z[2], z[1]],
            [z[2], 0, -z[0]],
            [-z[1], z[0], 0]
        ])
        
        # Jacobian of z-axis change: d(foot_z_world)/d(dq) = -skew(z) @ Jr
        Jz_axis = -skew_z @ Jr_act  # (3, nq_a)
        
        # We only care about xy components
        Jz_xy = Jz_axis[:2, :]  # (2, nq_a)
        
        # Linearized error: err_xy_new ≈ err_xy_0 + Jz_xy @ dqa
        err = err_xy_0 + Jz_xy @ dqa  # CVXPY expr (2,)
        
        total_cost += float(weight) * cp.sum_squares(err)

    return total_cost


def calc_rotation_jacobian_mujoco(
    robot_model,
    robot_data,
    body_id: int,
    build_transform_qdot_to_qvel: Callable[[], np.ndarray],
) -> np.ndarray:
    """
    Compute the rotational Jacobian for a body using MuJoCo.
    
    Returns Jr such that omega_world = Jr @ qdot
    
    Parameters
    ----------
    robot_model, robot_data:
        MuJoCo model/data
    body_id:
        Body ID in MuJoCo
    build_transform_qdot_to_qvel:
        Function that returns T matrix: v = T @ qdot
    
    Returns
    -------
    Jr: (3, nq) array
        Rotational Jacobian wrt qdot
    """
    nv = robot_model.nv
    nq = robot_model.nq
    
    # Get rotational Jacobian wrt generalized velocities
    Jp = np.zeros((3, nv), dtype=np.float64, order="C")
    Jr = np.zeros((3, nv), dtype=np.float64, order="C")
    
    # Use body position for the Jacobian computation
    point = robot_data.xpos[body_id].reshape(3, 1).astype(np.float64)
    mujoco.mj_jac(robot_model, robot_data, Jp, Jr, point, int(body_id))
    
    # Transform from qvel to qdot
    T = build_transform_qdot_to_qvel()  # (nv, nq)
    
    return Jr @ T  # (3, nq)

