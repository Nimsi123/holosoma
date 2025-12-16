from __future__ import annotations

from typing import Dict, Tuple, Optional, Sequence
import numpy as np
import cvxpy as cp

# Example: offsets in FOOT link/body frame
foot_sole_offsets = {
    "left_foot": np.array([
        [ 0.10,  0.04, -0.02],  # toe-left
        [ 0.10, -0.04, -0.02],  # toe-right
        [-0.08,  0.04, -0.02],  # heel-left
        [-0.08, -0.04, -0.02],  # heel-right
    ]),
    "right_foot": np.array([
        [ 0.10,  0.04, -0.02],
        [ 0.10, -0.04, -0.02],
        [-0.08,  0.04, -0.02],
        [-0.08, -0.04, -0.02],
    ]),
}
FOOT_FLAT_TOL_Z = 2e-3      # 2 mm
w_flat = 50.0          # if using a soft cost instead of hard constraints


def make_soft_flat_feet_cost(
    *,
    robot_model,
    robot_data,
    foot_links: Dict[str, str],
    foot_sticking_flags: Dict[str, bool],
    q_a_indices: np.ndarray,
    dqa: cp.Variable,
    # Optional: only activate if this is True (e.g., your activate_foot_sticking)
    active: bool = True,
    # Optional: which keys in foot_links map to which offsets
    # If None, auto-heuristic based on "left"/"right" substrings.
    left_right_map: Optional[Dict[str, str]] = None,
    # Hook into your class method that returns (3 x nq) Jacobian wrt qdot:
    calc_contact_jacobian_from_point=None,
) -> cp.Expression:
    """
    Soft flat-foot cost: encourage the sole to be parallel to the ground by
    penalizing differences in world z-height across multiple sample points on each foot.

    Adds a quadratic term:
        w_flat * || ( (z_i - z_0) + (Jz_i - Jz_0) dqa ) ||^2
    for each "sticking" foot, where i indexes sole sample points.

    Parameters
    ----------
    robot_model, robot_data:
        MuJoCo model/data (must already match the current q; i.e., mj_forward already done).
    foot_links:
        Mapping {key -> mujoco_body_name} for the feet (e.g., {"left_foot": "...", "right_foot": "..."}).
        The keys are used to decide left/right unless left_right_map is provided.
    foot_sticking_flags:
        Dict that tells which feet are planted, e.g. {"left": True, "right": False} or {"l": True, "r": False}.
        This function will infer left/right by prefixes if needed.
    q_a_indices:
        Actuated indices slice into q (same as self.q_a_indices).
    dqa:
        CVXPY decision variable for delta-actuated.
    foot_sole_offsets:
        Dict with arrays of shape (K,3), offsets in the foot BODY frame.
        Must include entries for left/right (names can be mapped with left_right_map).
    w_flat:
        Scalar weight for the flatness penalty.
    active:
        If False, returns 0.
    left_right_map:
        Optional mapping from foot_links key -> entry in foot_sole_offsets.
        Example: {"left_ankle": "left_foot", "right_ankle": "right_foot"}
    calc_contact_jacobian_from_point:
        Callable like: J = calc_contact_jacobian_from_point(body_id:int, p_body:np.ndarray) -> (3 x nq)

    Returns
    -------
    cp.Expression
        A CVXPY expression you can add to your objective (obj_terms.append(expr)).
    """
    if (not active) or (w_flat <= 0.0):
        return 0.0
    if calc_contact_jacobian_from_point is None:
        raise ValueError("You must pass calc_contact_jacobian_from_point (e.g., self._calc_contact_jacobian_from_point).")

    # Infer left/right flags from foot_sticking_flags
    def _flag(name: str) -> bool:
        # Accept "left"/"right", "l"/"r", etc.
        for k, v in foot_sticking_flags.items():
            kk = str(k).lower()
            if name == "left" and (kk.startswith("l") or "left" in kk):
                return bool(v)
            if name == "right" and (kk.startswith("r") or "right" in kk):
                return bool(v)
        return False

    left_on = _flag("left")
    right_on = _flag("right")

    # Build cost
    total_cost = 0.0

    for foot_key, body_name in foot_links.items():
        foot_key_l = foot_key.lower()
        is_left = ("left" in foot_key_l) or foot_key_l.startswith("l")
        is_right = ("right" in foot_key_l) or foot_key_l.startswith("r")
        apply = (is_left and left_on) or (is_right and right_on)
        if not apply:
            continue

        # Choose which offsets array to use
        if left_right_map is not None:
            offsets_name = left_right_map[foot_key]
        else:
            offsets_name = "left_foot" if is_left else "right_foot"

        if offsets_name not in foot_sole_offsets:
            raise KeyError(f"foot_sole_offsets is missing key '{offsets_name}'. Available: {list(foot_sole_offsets.keys())}")

        offsets = np.asarray(foot_sole_offsets[offsets_name], dtype=float)  # (K,3)
        if offsets.ndim != 2 or offsets.shape[1] != 3 or offsets.shape[0] < 2:
            raise ValueError(f"Offsets for '{offsets_name}' must have shape (K,3) with K>=2; got {offsets.shape}")

        # Get MuJoCo body id
        import mujoco  # local import to keep this file importable without mujoco installed
        body_id = mujoco.mj_name2id(robot_model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if body_id < 0:
            raise ValueError(f"Body '{body_name}' (from foot_links['{foot_key}']) not found in MuJoCo model.")

        # Current body pose
        R_WB = robot_data.xmat[body_id].reshape(3, 3)
        p_WB = robot_data.xpos[body_id].reshape(3)

        # Collect z0 and Jz for each point
        K = offsets.shape[0]
        z0 = np.empty(K, dtype=float)
        Jz = np.empty((K, len(q_a_indices)), dtype=float)

        for i in range(K):
            pC_B = offsets[i]
            # World point position at current q
            p_W = p_WB + R_WB @ pC_B
            z0[i] = float(p_W[2])

            # Jacobian row for world z wrt qdot, then slice to actuated indices
            Jp = calc_contact_jacobian_from_point(body_id, pC_B)  # (3 x nq)
            Jz[i, :] = np.asarray(Jp[2, q_a_indices], dtype=float)

        # Error: (z_i - z_0) + (Jz_i - Jz_0) dqa for i=1..K-1
        dz0 = z0[1:] - z0[0]                      # (K-1,)
        dJ = Jz[1:, :] - Jz[0:1, :]               # (K-1, nq_a)
        err = dz0 + dJ @ dqa                      # CVXPY expr (K-1,)

        total_cost += float(w_flat) * cp.sum_squares(err)

    return total_cost
