"""
sim/environment.py: 3D Disaster Environment, Spatial Bounds, Altitude Corridors & GCS Anchor.

Implements coordinate validation, boundary clamping, and 4-tier airspace deconfliction
for the 500x500x120m disaster operations theater.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np


class AltitudeCorridor(str, Enum):
    """Airspace deconfliction layers for disaster response operations."""
    GROUND_LAUNCH_LAND = "LAUNCH_LAND"  # [0, 20]m
    POI_SURVEY = "POI_SURVEY"            # [25, 45]m
    TRANSIT = "TRANSIT"                  # [50, 65]m
    RELAY_MESH = "RELAY_MESH"            # [70, 90]m
    BUFFER_ZONE = "BUFFER_ZONE"          # Transition bands between corridors
    ABOVE_CEILING = "ABOVE_CEILING"      # > 120m
    BELOW_GROUND = "BELOW_GROUND"        # < 0m


@dataclass
class EnvironmentConfig:
    """Configurable boundaries and base stations for the disaster theater."""
    x_min: float = -500.0
    x_max: float = 500.0
    y_min: float = -650.0
    y_max: float = 500.0
    z_min: float = 0.0
    z_max: float = 130.0

    # Ground Control Station location (default origin)
    gcs_position: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0, 0.0], dtype=np.float64))

    # GCS RF antenna height above ground (meters)
    gcs_antenna_height: float = 2.5

    # Altitude corridors [min, max] in meters
    layer_launch_land: Tuple[float, float] = (0.0, 20.0)
    layer_poi_survey: Tuple[float, float] = (25.0, 45.0)
    layer_transit: Tuple[float, float] = (50.0, 65.0)
    layer_relay_mesh: Tuple[float, float] = (70.0, 90.0)

    def __post_init__(self) -> None:
        self.gcs_position = np.asarray(self.gcs_position, dtype=np.float64)
        if self.x_min >= self.x_max or self.y_min >= self.y_max or self.z_min >= self.z_max:
            raise ValueError(
                f"Invalid environment bounds: X=[{self.x_min}, {self.x_max}], "
                f"Y=[{self.y_min}, {self.y_max}], Z=[{self.z_min}, {self.z_max}]"
            )


class DisasterEnvironment:
    """
    Owner of operational spatial bounds, altitude layering, and GCS positioning.
    Provides vectorized coordinate validation, boundary clamping, and safety enforcement.
    """

    def __init__(
        self,
        config: Optional[EnvironmentConfig] = None,
        bounds_x: Optional[Tuple[float, float]] = None,
        bounds_y: Optional[Tuple[float, float]] = None,
        bounds_z: Optional[Tuple[float, float]] = None,
        gcs_position: Optional[Union[np.ndarray, Sequence[float]]] = None,
    ) -> None:
        if config is not None:
            self.config = config
        else:
            x_min, x_max = bounds_x if bounds_x is not None else (-250.0, 250.0)
            y_min, y_max = bounds_y if bounds_y is not None else (-250.0, 250.0)
            z_min, z_max = bounds_z if bounds_z is not None else (0.0, 120.0)
            gcs_pos = np.array(gcs_position if gcs_position is not None else [0.0, 0.0, 0.0], dtype=np.float64)
            self.config = EnvironmentConfig(
                x_min=x_min,
                x_max=x_max,
                y_min=y_min,
                y_max=y_max,
                z_min=z_min,
                z_max=z_max,
                gcs_position=gcs_pos,
            )

        # Cache min and max bound vectors for rapid vectorized clamping
        self._min_bound = np.array([self.config.x_min, self.config.y_min, self.config.z_min], dtype=np.float64)
        self._max_bound = np.array([self.config.x_max, self.config.y_max, self.config.z_max], dtype=np.float64)
        self.obstacles: List[Any] = []

    # -------------------------------------------------------------------------
    # Spatial Properties
    # -------------------------------------------------------------------------

    @property
    def gcs_position(self) -> np.ndarray:
        """Returns stationary 3D ground location of GCS."""
        return self.config.gcs_position.copy()

    @property
    def gcs_rf_position(self) -> np.ndarray:
        """Returns 3D phase center of GCS communication antenna mast."""
        rf_pos = self.config.gcs_position.copy()
        rf_pos[2] += self.config.gcs_antenna_height
        return rf_pos

    @property
    def width_x(self) -> float:
        """Total width along East-West X-axis in meters."""
        return self.config.x_max - self.config.x_min

    @property
    def length_y(self) -> float:
        """Total length along North-South Y-axis in meters."""
        return self.config.y_max - self.config.y_min

    @property
    def height_z(self) -> float:
        """Total vertical altitude clearance in meters."""
        return self.config.z_max - self.config.z_min

    @property
    def bounds_min(self) -> np.ndarray:
        """Lower boundary corner [X_min, Y_min, Z_min]."""
        return self._min_bound.copy()

    @property
    def bounds_max(self) -> np.ndarray:
        """Upper boundary corner [X_max, Y_max, Z_max]."""
        return self._max_bound.copy()

    @property
    def bounds_x(self) -> Tuple[float, float]:
        """Tuple (X_min, X_max)."""
        return (self.config.x_min, self.config.x_max)

    @property
    def bounds_y(self) -> Tuple[float, float]:
        """Tuple (Y_min, Y_max)."""
        return (self.config.y_min, self.config.y_max)

    @property
    def bounds_z(self) -> Tuple[float, float]:
        """Tuple (Z_min, Z_max)."""
        return (self.config.z_min, self.config.z_max)

    # -------------------------------------------------------------------------
    # Boundary Checking & Clamping
    # -------------------------------------------------------------------------

    def is_within_bounds(self, point: np.ndarray, margin: float = 0.0) -> bool:
        """
        Determines whether point [x, y, z] is inside the active environment volume.
        An optional inward margin shrinks the allowable bounding box.
        """
        p = np.asarray(point, dtype=np.float64)
        if p.shape != (3,):
            raise ValueError(f"Expected 3D point shape (3,), got {p.shape}")

        return bool(
            (p[0] >= self.config.x_min + margin)
            and (p[0] <= self.config.x_max - margin)
            and (p[1] >= self.config.y_min + margin)
            and (p[1] <= self.config.y_max - margin)
            and (p[2] >= self.config.z_min)
            and (p[2] <= self.config.z_max - margin)
        )

    def clamp_to_bounds(self, point: np.ndarray, margin: float = 0.0) -> np.ndarray:
        """Clamps a 3D coordinate vector to stay within allowable boundary volume."""
        p = np.asarray(point, dtype=np.float64)
        min_eff = self._min_bound + np.array([margin, margin, 0.0])
        max_eff = self._max_bound - margin
        return np.clip(p, min_eff, max_eff)

    def clamp_position(self, point: np.ndarray, margin: float = 0.0) -> np.ndarray:
        """Alias for clamp_to_bounds."""
        return self.clamp_to_bounds(point, margin=margin)

    def enforce_bounds(self, drone: Any) -> None:
        """
        Enforces physical bounds and ground collision on a drone agent.
        Clamps horizontal bounds and sets z=0 if ground contact is detected.
        """
        pos = drone.position
        # Horizontal clamp
        pos[0] = max(self.config.x_min, min(self.config.x_max, pos[0]))
        pos[1] = max(self.config.y_min, min(self.config.y_max, pos[1]))
        pos[2] = min(self.config.z_max, pos[2])

        # Ground collision hard constraint (Z >= 0)
        if pos[2] <= 0.0:
            pos[2] = 0.0
            if hasattr(drone, "velocity"):
                drone.velocity[2] = max(0.0, float(drone.velocity[2]))
            if hasattr(drone, "acceleration"):
                drone.acceleration[2] = max(0.0, float(drone.acceleration[2]))

    def validate_position(self, point: np.ndarray) -> None:
        """Raises ValueError if coordinates contain NaN, Inf, or breach environment boundaries."""
        p = np.asarray(point, dtype=np.float64)
        if not np.all(np.isfinite(p)):
            raise ValueError(f"Position contains non-finite values (NaN/Inf): {p}")
        if not self.is_within_bounds(p):
            raise ValueError(
                f"Position {p} is outside bounds X:[{self.config.x_min}, {self.config.x_max}], "
                f"Y:[{self.config.y_min}, {self.config.y_max}], Z:[{self.config.z_min}, {self.config.z_max}]"
            )

    def distance_to_boundary(self, point: np.ndarray) -> float:
        """
        Returns minimum distance from 3D point to closest outer boundary plane.
        Positive value indicates inside the boundary; negative indicates outside.
        """
        p = np.asarray(point, dtype=np.float64)
        dists = [
            p[0] - self.config.x_min,
            self.config.x_max - p[0],
            p[1] - self.config.y_min,
            self.config.y_max - p[1],
            p[2] - self.config.z_min,
            self.config.z_max - p[2],
        ]
        return float(min(dists))

    def get_altitude_corridor(self, altitude: float) -> AltitudeCorridor:
        """Classifies a given altitude Z (AGL in meters) into its operational corridor."""
        z = float(altitude)
        if z < self.config.z_min:
            return AltitudeCorridor.BELOW_GROUND
        elif z > self.config.z_max:
            return AltitudeCorridor.ABOVE_CEILING
        elif self.config.layer_launch_land[0] <= z <= self.config.layer_launch_land[1]:
            return AltitudeCorridor.GROUND_LAUNCH_LAND
        elif self.config.layer_poi_survey[0] <= z <= self.config.layer_poi_survey[1]:
            return AltitudeCorridor.POI_SURVEY
        elif self.config.layer_transit[0] <= z <= self.config.layer_transit[1]:
            return AltitudeCorridor.TRANSIT
        elif self.config.layer_relay_mesh[0] <= z <= self.config.layer_relay_mesh[1]:
            return AltitudeCorridor.RELAY_MESH
        else:
            return AltitudeCorridor.BUFFER_ZONE

    def add_obstacle(self, obstacle: Any) -> None:
        """Registers a static obstacle in the environment."""
        self.obstacles.append(obstacle)

    def to_dict(self) -> Dict[str, Any]:
        """Serializes environment configuration into JSON-compatible dictionary."""
        return {
            "x_bounds": [self.config.x_min, self.config.x_max],
            "y_bounds": [self.config.y_min, self.config.y_max],
            "z_bounds": [self.config.z_min, self.config.z_max],
            "dimensions_m": [self.width_x, self.length_y, self.height_z],
            "gcs_position": self.gcs_position.tolist(),
            "gcs_rf_position": self.gcs_rf_position.tolist(),
            "altitude_corridors": {
                "launch_land": list(self.config.layer_launch_land),
                "poi_survey": list(self.config.layer_poi_survey),
                "transit": list(self.config.layer_transit),
                "relay_mesh": list(self.config.layer_relay_mesh),
                "ceiling": self.config.z_max,
            },
        }
