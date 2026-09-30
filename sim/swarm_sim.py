"""
sim/swarm_sim.py: Accelerated Swarm Spatial Hash, Proximity Queries & Vectorized Dynamics.

Implements:
1. 3D Spatial Grid / Hash for O(1) drone-drone proximity and collision avoidance queries.
2. Batched vectorized swarm steering forces.
3. Re-exports SwarmSimulationCore for unified modular access.
"""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Any, Dict, List, Optional, Sequence, Set, Tuple
import numpy as np

if TYPE_CHECKING:
    from sim.core import SimulationConfig, SwarmSimulationCore

from sim.physics import batched_drone_aerodynamic_drag, batched_pairwise_distances


def __getattr__(name: str) -> Any:
    if name in ("SimulationConfig", "SwarmSimulationCore"):
        import sim.core as core_mod
        return getattr(core_mod, name)
    raise AttributeError(f"module '{__name__}' has no attribute '{name}'")


class DroneSpatialGrid:
    """
    3D Spatial Grid for accelerated drone-drone proximity and collision avoidance queries.
    Partitions 3D space into cubic bins of side length cell_size (e.g. 20.0m).
    Replaces O(N^2) exhaustive pairwise distance checks with O(1) amortized neighborhood lookups.
    """

    def __init__(self, cell_size: float = 20.0) -> None:
        self.cell_size = float(cell_size)
        self.inv_cell = 1.0 / self.cell_size
        self.grid: Dict[Tuple[int, int, int], List[Any]] = {}

    def clear(self) -> None:
        """Clear all active spatial bins."""
        self.grid.clear()

    def insert(self, drone: Any) -> None:
        """Insert a single drone into its corresponding spatial grid cell."""
        pos = getattr(drone, "position", drone)
        key = (
            int(math.floor(pos[0] * self.inv_cell)),
            int(math.floor(pos[1] * self.inv_cell)),
            int(math.floor(pos[2] * self.inv_cell)),
        )
        self.grid.setdefault(key, []).append(drone)

    def build(self, drones: Sequence[Any]) -> None:
        """Reconstruct spatial grid for fleet of drones."""
        self.clear()
        for d in drones:
            self.insert(d)

    def query_radius(self, position: np.ndarray, radius: float, exclude_id: Optional[str] = None) -> List[Any]:
        """
        Query all drones within radius meters of position.
        Tests only candidate drones within intersecting spatial grid cells.
        """
        r_cells = int(math.ceil(radius * self.inv_cell))
        cx = int(math.floor(position[0] * self.inv_cell))
        cy = int(math.floor(position[1] * self.inv_cell))
        cz = int(math.floor(position[2] * self.inv_cell))
        r_sq = radius * radius
        results = []

        for dx in range(-r_cells, r_cells + 1):
            for dy in range(-r_cells, r_cells + 1):
                for dz in range(-r_cells, r_cells + 1):
                    cell = self.grid.get((cx + dx, cy + dy, cz + dz))
                    if not cell:
                        continue
                    for d in cell:
                        d_id = getattr(d, "id", None)
                        if exclude_id and d_id == exclude_id:
                            continue
                        pos = getattr(d, "position", d)
                        dist_sq = float(np.sum((pos - position) ** 2))
                        if dist_sq <= r_sq:
                            results.append(d)
        return results

    def query_nearby_peers(self, drone: Any, search_radius: float = 25.0) -> List[Any]:
        """Convenience method to query neighboring drones around a reference drone."""
        pos = getattr(drone, "position", np.zeros(3))
        d_id = getattr(drone, "id", None)
        return self.query_radius(pos, search_radius, exclude_id=d_id)


def batched_reynolds_separation_forces(
    positions: np.ndarray,
    velocities: np.ndarray,
    r_sep: float = 6.0,
    k_sep: float = 45.0,
) -> np.ndarray:
    """
    Vectorized Reynolds inter-drone separation force calculation across an N x 3 fleet.
    Computes repulsive forces between all drone pairs within r_sep distance in vectorized tensor time.

    Args:
        positions: Array of shape [N, 3] representing coordinates of N drones.
        velocities: Array of shape [N, 3] representing velocity vectors.
        r_sep: Separation distance horizon threshold.
        k_sep: Repulsive force scaling coefficient.

    Returns:
        f_sep: Array of shape [N, 3] of net separation forces acting on each drone.
    """
    n = len(positions)
    if n <= 1:
        return np.zeros((n, 3), dtype=np.float64)

    diff, dists = batched_pairwise_distances(positions)
    np.fill_diagonal(dists, np.inf)

    interact_mask = (dists > 1e-4) & (dists < r_sep)
    safe_dists = np.where(dists > 1e-4, dists, 1.0)[:, :, np.newaxis]
    unit_repulse = diff / safe_dists

    mag = np.zeros_like(dists)
    valid_dists = dists[interact_mask]
    d_eff = np.maximum(valid_dists - 1.8, 0.1)
    r_eff = max(r_sep - 1.8, 0.2)
    mag[interact_mask] = k_sep * (1.0 / d_eff - 1.0 / r_eff) / (d_eff ** 2)
    mag = np.clip(mag, 0.0, 250.0)[:, :, np.newaxis]

    f_sep = np.sum(unit_repulse * mag * interact_mask[:, :, np.newaxis], axis=1)
    return f_sep


def batched_update_kinematics(
    positions: np.ndarray,
    velocities: np.ndarray,
    accelerations: np.ndarray,
    forces: np.ndarray,
    masses: np.ndarray,
    dt: float,
    max_speeds: Optional[np.ndarray] = None,
    max_accels: Optional[np.ndarray] = None,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Vectorized semi-implicit Euler integration of translational kinematics across an N-drone swarm.

    Args:
        positions: Array of shape [N, 3] of current positions.
        velocities: Array of shape [N, 3] of current velocities.
        accelerations: Array of shape [N, 3] of current accelerations.
        forces: Array of shape [N, 3] of net forces acting on drones.
        masses: Array of shape [N] of vehicle masses (kg).
        dt: Time step in seconds.
        max_speeds: Optional array of shape [N] of max speeds (m/s).
        max_accels: Optional array of shape [N] of max accelerations (m/s^2).

    Returns:
        (new_positions, new_velocities, new_accelerations)
    """
    if len(positions) == 0:
        return positions.copy(), velocities.copy(), accelerations.copy()

    inv_m = 1.0 / np.maximum(masses[:, np.newaxis], 1e-3)
    raw_accel = forces * inv_m

    if max_accels is not None:
        a_norms = np.linalg.norm(raw_accel, axis=-1, keepdims=True)
        a_max = max_accels[:, np.newaxis]
        scale_a = np.where(a_norms > a_max, a_max / np.maximum(a_norms, 1e-6), 1.0)
        new_accel = raw_accel * scale_a
    else:
        new_accel = raw_accel

    new_vel = velocities + new_accel * dt

    if max_speeds is not None:
        v_norms = np.linalg.norm(new_vel, axis=-1, keepdims=True)
        v_max = max_speeds[:, np.newaxis]
        scale_v = np.where(v_norms > v_max, v_max / np.maximum(v_norms, 1e-6), 1.0)
        new_vel = new_vel * scale_v

    new_pos = positions + new_vel * dt
    return new_pos, new_vel, new_accel
