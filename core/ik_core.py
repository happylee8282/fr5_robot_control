"""Pure IK branch scoring utilities, independent from ROS."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np


@dataclass
class BranchState:
    q: np.ndarray
    previous_delta: np.ndarray
    total_cost: float
    parent: "BranchState | None"
    pose_index: int


def equivalent_near_reference(
    candidate: Sequence[float],
    reference: Sequence[float],
    lower: Sequence[float],
    upper: Sequence[float],
) -> np.ndarray | None:
    """Choose the legal 2-pi equivalent closest to the preceding state."""

    candidate = np.asarray(candidate, float)
    reference = np.asarray(reference, float)
    lower = np.asarray(lower, float)
    upper = np.asarray(upper, float)
    result = np.empty_like(candidate)
    for index, value in enumerate(candidate):
        equivalents = value + 2.0 * np.pi * np.arange(-3, 4)
        legal = equivalents[(equivalents >= lower[index]) & (equivalents <= upper[index])]
        if not len(legal):
            return None
        result[index] = legal[np.argmin(np.abs(legal - reference[index]))]
    return result


def incremental_branch_cost(
    candidate: np.ndarray,
    previous: np.ndarray,
    previous_delta: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    weights: Mapping[str, float],
) -> float:
    """Penalize motion, curvature, limits, and the UR wrist singularity."""

    delta = candidate - previous
    joint_weights = np.array([1.0, 1.2, 1.1, 0.7, 0.8, 0.5])
    motion = float(np.sum(joint_weights * delta**2))
    curvature = float(np.sum((delta - previous_delta) ** 2))
    span = np.maximum(upper - lower, 1.0e-9)
    margin = np.minimum(candidate - lower, upper - candidate) / span
    limit = float(np.sum(1.0 / np.maximum(margin, 0.02) ** 2))
    wrist = float(1.0 / (np.sin(candidate[4]) ** 2 + 0.02))
    return (
        float(weights["joint_motion"]) * motion
        + float(weights["joint_acceleration"]) * curvature
        + float(weights["joint_limit"]) * limit
        + float(weights["wrist_singularity"]) * wrist
    )


def reconstruct_branch(state: BranchState) -> np.ndarray:
    """Follow parent pointers from the selected final branch."""

    values = []
    current: BranchState | None = state
    while current is not None and current.pose_index >= 0:
        values.append(current.q)
        current = current.parent
    values.reverse()
    return np.asarray(values, dtype=float)
