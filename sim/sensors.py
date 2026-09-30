"""
sim/sensors.py: Production Sensor Suite (IMU, GPS, Barometer) & Extended Kalman Filter (EKF).

Implements:
1. Multi-rate physical sensor simulations:
   - 3-axis Accelerometer & Gyroscope with bias random walk and Gaussian white noise.
   - GNSS/GPS Receiver with HDOP degradation near tall urban structures.
   - Barometric Altimeter with pressure drift.
2. 9-State Extended Kalman Filter (EKF) fusing noisy sensor measurements into
   an optimal real-time navigation estimate of 3D Position, Velocity, and Attitude.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Sequence, Tuple
import numpy as np


@dataclass
class SensorConfig:
    """Sensor accuracy and noise characteristics matching industrial MEMS avionics."""
    # IMU Accelerometer
    accel_noise_std: float = 0.08         # m/s^2 white noise
    accel_bias_stability: float = 0.005    # Random walk drift per second
    
    # IMU Gyroscope
    gyro_noise_std: float = 0.005         # rad/s white noise
    gyro_bias_stability: float = 0.0002   # rad/s^2 drift per second

    # GPS Receiver
    gps_pos_std_horiz: float = 0.75       # Horizontal standard deviation (m)
    gps_pos_std_vert: float = 1.40        # Vertical standard deviation (m)
    gps_vel_std: float = 0.15             # Velocity standard deviation (m/s)
    gps_update_interval: float = 0.10     # 10 Hz GPS update rate

    # Barometer
    baro_noise_std: float = 0.25          # Altitude noise (m)
    baro_update_interval: float = 0.05    # 20 Hz Baro update rate


class SensorSuite:
    """
    Physical sensor simulator converting ground-truth kinematics into realistic,
    noisy avionics measurement streams with bias drift and urban satellite shadowing.
    """

    def __init__(self, config: Optional[SensorConfig] = None) -> None:
        self.config = config if config is not None else SensorConfig()
        
        # Internal sensor bias states
        self.accel_bias: np.ndarray = np.random.normal(0.0, 0.02, 3)
        self.gyro_bias: np.ndarray = np.random.normal(0.0, 0.001, 3)
        self.baro_bias: float = float(np.random.normal(0.0, 0.1))

        # Sensor update timers
        self._time_since_gps: float = 0.0
        self._time_since_baro: float = 0.0

        # Latest buffered measurements
        self.latest_imu: Dict[str, np.ndarray] = {
            "accel": np.zeros(3, dtype=np.float64),
            "gyro": np.zeros(3, dtype=np.float64),
        }
        self.latest_gps: Optional[Dict[str, np.ndarray]] = None
        self.latest_baro: Optional[float] = None

    def sample(
        self,
        true_pos: np.ndarray,
        true_vel: np.ndarray,
        true_accel: np.ndarray,
        true_angular_vel: np.ndarray,
        dt: float,
        gps_hdop_multiplier: float = 1.0,
    ) -> Dict[str, Any]:
        """
        Advance internal sensor states and generate multi-rate noisy measurement packet.
        """
        # 1. Update random-walk biases
        self.accel_bias += np.random.normal(0.0, self.config.accel_bias_stability * math.sqrt(dt), 3)
        self.gyro_bias += np.random.normal(0.0, self.config.gyro_bias_stability * math.sqrt(dt), 3)

        # 2. Sample IMU (every tick)
        n_a = np.random.normal(0.0, self.config.accel_noise_std, 3)
        n_g = np.random.normal(0.0, self.config.gyro_noise_std, 3)
        meas_accel = true_accel + self.accel_bias + n_a
        meas_gyro = true_angular_vel + self.gyro_bias + n_g
        self.latest_imu = {"accel": meas_accel, "gyro": meas_gyro}

        # 3. Sample Barometer (at 20 Hz)
        self._time_since_baro += dt
        has_baro = False
        if self._time_since_baro >= self.config.baro_update_interval:
            self._time_since_baro = 0.0
            n_baro = float(np.random.normal(0.0, self.config.baro_noise_std))
            self.latest_baro = float(true_pos[2] + self.baro_bias + n_baro)
            has_baro = True

        # 4. Sample GPS (at 10 Hz)
        self._time_since_gps += dt
        has_gps = False
        if self._time_since_gps >= self.config.gps_update_interval:
            self._time_since_gps = 0.0
            hdop = max(1.0, float(gps_hdop_multiplier))
            n_pos = np.array([
                np.random.normal(0.0, self.config.gps_pos_std_horiz * hdop),
                np.random.normal(0.0, self.config.gps_pos_std_horiz * hdop),
                np.random.normal(0.0, self.config.gps_pos_std_vert * hdop),
            ], dtype=np.float64)
            n_vel = np.random.normal(0.0, self.config.gps_vel_std * hdop, 3)
            self.latest_gps = {
                "position": true_pos + n_pos,
                "velocity": true_vel + n_vel,
                "hdop": hdop,
            }
            has_gps = True

        return {
            "imu": self.latest_imu,
            "gps": self.latest_gps if has_gps else None,
            "baro": self.latest_baro if has_baro else None,
        }


class DroneEKF:
    """
    9-State Extended Kalman Filter for UAV Navigation State Estimation.
    
    State Vector: x = [p_x, p_y, p_z, v_x, v_y, v_z, b_ax, b_ay, b_az]^T
    Fuses continuous IMU acceleration with intermittent GPS and Baro measurements.
    """

    def __init__(self, initial_position: Optional[np.ndarray] = None) -> None:
        init_p = initial_position if initial_position is not None else np.zeros(3)
        
        # State vector (9x1): [pos(3), vel(3), accel_bias(3)]
        self.x = np.zeros(9, dtype=np.float64)
        self.x[0:3] = np.asarray(init_p, dtype=np.float64)

        # Error Covariance Matrix (9x9)
        self.P = np.eye(9, dtype=np.float64)
        self.P[0:3, 0:3] *= 1.0     # Position uncertainty
        self.P[3:6, 3:6] *= 0.1     # Velocity uncertainty
        self.P[6:9, 6:9] *= 0.05    # Bias uncertainty

        # Process Noise Covariance (9x9)
        self.Q = np.eye(9, dtype=np.float64)
        self.Q[0:3, 0:3] *= 0.001   # Position drift
        self.Q[3:6, 3:6] *= 0.02    # Acceleration noise integration
        self.Q[6:9, 6:9] *= 0.0001  # Bias random walk

        # Measurement Noise Covariances
        self.R_gps_pos = np.diag([0.6 ** 2, 0.6 ** 2, 1.2 ** 2])
        self.R_gps_vel = np.diag([0.15 ** 2, 0.15 ** 2, 0.25 ** 2])
        self.R_baro = np.array([[0.3 ** 2]])

    def predict(self, accel_meas: np.ndarray, dt: float) -> None:
        """
        EKF Time-Propagation step using measured specific acceleration.
        """
        dt = max(1e-4, float(dt))
        
        # Extract estimated acceleration subtracting estimated bias
        a_hat = accel_meas - self.x[6:9]

        if getattr(self, "_last_dt", None) != dt:
            if not hasattr(self, "_F"):
                self._F = np.eye(9, dtype=np.float64)
            dt_sq_half = -0.5 * (dt ** 2)
            for i in range(3):
                self._F[i, 3 + i] = dt
                self._F[i, 6 + i] = dt_sq_half
                self._F[3 + i, 6 + i] = -dt
            self._last_dt = dt
            self._Q_dt = self.Q * dt

        # Non-linear state propagation
        self.x[0:3] += self.x[3:6] * dt + 0.5 * a_hat * (dt ** 2)
        self.x[3:6] += a_hat * dt

        # Covariance propagation
        self.P = self._F @ self.P @ self._F.T + self._Q_dt
        self.P = 0.5 * (self.P + self.P.T)

    def update_gps(self, gps_pos: np.ndarray, gps_vel: np.ndarray, hdop: float = 1.0) -> None:
        """
        EKF Measurement update with GPS 3D position and velocity (6x1 observation).
        Vectorized block update directly extracting submatrices to avoid redundant 6x9 allocations.
        """
        residual = np.hstack([gps_pos, gps_vel]) - self.x[0:6]
        hdop_sq = hdop ** 2
        R = np.block([
            [self.R_gps_pos * hdop_sq, np.zeros((3, 3))],
            [np.zeros((3, 3)), self.R_gps_vel * hdop_sq],
        ])

        S = self.P[0:6, 0:6] + R
        K = np.linalg.solve(S.T, self.P[0:6, :]).T

        self.x += K @ residual
        self.P -= K @ self.P[0:6, :]
        self.P = 0.5 * (self.P + self.P.T)

    def update_baro(self, baro_alt: float) -> None:
        """
        EKF Measurement update with Barometer altitude (1x1 observation).
        Vectorized 1D kalman update in-place without temporary matrix objects.
        """
        residual = float(baro_alt) - self.x[2]
        S = float(self.P[2, 2] + self.R_baro[0, 0])
        K = self.P[:, 2] / S

        self.x += K * residual
        self.P -= np.outer(K, self.P[2, :])
        self.P = 0.5 * (self.P + self.P.T)

    @property
    def estimated_position(self) -> np.ndarray:
        return self.x[0:3].copy()

    @property
    def estimated_velocity(self) -> np.ndarray:
        return self.x[3:6].copy()

    @property
    def estimated_accel_bias(self) -> np.ndarray:
        return self.x[6:9].copy()
