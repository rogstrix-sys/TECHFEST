"""
sim/physics.py: Aerodynamic Physics, Stochastic Wind Turbulence & Vectorized Dynamics.

Implements:
1. Continuous-time Ornstein-Uhlenbeck stochastic wind process for atmospheric turbulence.
2. Aerodynamic drag and downwash wake interaction models.
3. Batched vectorized quadcopter kinematics utilities.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np


class OrnsteinUhlenbeckWind:
    """
    Continuous-time Ornstein-Uhlenbeck stochastic process for realistic aerodynamic wind & turbulence.
    
    Differential Equation:
        d v(t) = -theta * (v(t) - mu) * dt + sigma * sqrt(dt) * dW(t)
    
    Properties:
    - Mean-reverting to ambient vector mu.
    - Temporally correlated (correlation time tau = 1 / theta).
    - Gaussian velocity perturbations matching atmospheric boundary layer turbulence.
    """

    def __init__(
        self,
        mean_velocity: Optional[Sequence[float]] = None,
        theta: float = 0.6,
        sigma: float = 1.4,
        seed: Optional[int] = None,
    ) -> None:
        self.mu = np.array(mean_velocity if mean_velocity is not None else [0.0, 0.0, 0.0], dtype=np.float64)
        self.theta = float(theta)
        self.sigma = float(sigma)
        self.rng = np.random.default_rng(seed)
        self.state = self.mu.copy()

    def set_mean_velocity(self, mean_vel: Sequence[float]) -> None:
        """Update mean horizontal/vertical wind baseline."""
        self.mu = np.array(mean_vel, dtype=np.float64)

    def step(self, dt: float = 0.05) -> np.ndarray:
        """
        Advance the Ornstein-Uhlenbeck process by time step dt.
        Returns the updated 3D stochastic wind velocity vector [vx, vy, vz].
        """
        dt_clamped = max(1e-4, min(float(dt), 0.5))
        # Exact Euler-Maruyama discretization of OU process
        drift = -self.theta * (self.state - self.mu) * dt_clamped
        diffusion = self.sigma * math.sqrt(dt_clamped) * self.rng.standard_normal(3)
        self.state += drift + diffusion
        return self.state.copy()

    def sample_at_altitude(self, altitude_m: float, dt: float = 0.05) -> np.ndarray:
        """
        Samples stochastic wind with vertical boundary layer shear scaling.
        Wind speed scales with height z according to atmospheric power law (alpha = 0.143).
        """
        base = self.step(dt=dt)
        z = max(0.5, float(altitude_m))
        shear_factor = (z / 10.0) ** 0.143
        res = base.copy()
        res[:2] *= min(3.0, max(0.5, shear_factor))
        return res

    def reset(self) -> None:
        """Reset state to mean baseline."""
        self.state = self.mu.copy()


def batched_pairwise_distances(positions: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """
    Vectorized N x N pairwise displacement and Euclidean distance computation.
    
    Args:
        positions: Array of shape [N, 3] representing coordinates of N drones.
        
    Returns:
        diff: Array of shape [N, N, 3] where diff[i, j] = pos[i] - pos[j]
        dists: Array of shape [N, N] where dists[i, j] = ||pos[i] - pos[j]||
    """
    if len(positions) == 0:
        return np.zeros((0, 0, 3)), np.zeros((0, 0))
    # diff[i, j] = positions[i] - positions[j]
    diff = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
    dists = np.linalg.norm(diff, axis=-1)
    return diff, dists


def batched_drone_aerodynamic_drag(
    velocities: np.ndarray,
    ambient_wind: np.ndarray,
    mass_kg: float = 1.5,
    drag_coeff_xy: float = 0.25,
    drag_coeff_z: float = 0.35,
) -> np.ndarray:
    """
    Vectorized aerodynamic drag acceleration calculation across an entire fleet.
    
    Args:
        velocities: Array of shape [N, 3] of drone inertial velocities.
        ambient_wind: Vector of shape [3] or array of shape [N, 3] of ambient wind.
        mass_kg: Quadcopter vehicle mass.
        drag_coeff_xy: Horizontal drag coefficient (N/(m/s)).
        drag_coeff_z: Vertical drag coefficient (N/(m/s)).
        
    Returns:
        drag_accel: Array of shape [N, 3] of decelerating drag accelerations.
    """
    if len(velocities) == 0:
        return np.zeros((0, 3))
    v_air = velocities - ambient_wind
    inv_m = 1.0 / max(mass_kg, 1e-3)
    drag_x = (drag_coeff_xy * inv_m) * v_air[:, 0]
    drag_y = (drag_coeff_xy * inv_m) * v_air[:, 1]
    drag_z = (drag_coeff_z * inv_m) * v_air[:, 2]
    return np.column_stack([drag_x, drag_y, drag_z])
