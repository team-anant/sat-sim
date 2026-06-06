import numpy as np

from .monitor import AnalysisMonitor
from ..ground import GroundStation
from ..maths.maths import eci_to_ecef


class TtcContactMonitor(AnalysisMonitor):
    def __init__(
        self,
        ground_station: GroundStation,
        min_elevation_deg: float | None = None,
        sample_period_s: float | None = None,
        antenna_component_name: str | None = "ttc_antenna",
        name: str = "ttc_contact",
    ):
        super().__init__(name)
        self.ground_station = ground_station
        self.min_elevation_deg = (
            None if min_elevation_deg is None else float(min_elevation_deg)
        )
        self.min_elevation_rad = (
            None if min_elevation_deg is None else np.radians(self.min_elevation_deg)
        )
        self.sample_period_s = sample_period_s
        self.antenna_component_name = antenna_component_name

        self.times = []
        self.elevations_rad = []
        self.in_contact = []

    def attach(self, sat):
        self._resolve_min_elevation(sat)

    @property
    def elevations_deg(self):
        return np.degrees(self.elevations_rad)

    @property
    def total_contact_time_s(self):
        if self.sample_period_s is not None:
            return float(np.sum(self.in_contact) * self.sample_period_s)

        if len(self.times) < 2:
            return 0.0

        times = np.asarray(self.times)
        in_contact = np.asarray(self.in_contact, dtype=float)
        dt = np.diff(times, prepend=times[0])
        return float(np.sum(in_contact * dt))

    @property
    def contact_fraction(self):
        return float(np.mean(self.in_contact)) if self.in_contact else 0.0

    def sample(self, sat):
        min_elevation_rad = self._resolve_min_elevation(sat)
        elevation = self.elevation_angle(sat.position, sat.time)
        self.times.append(float(sat.time))
        self.elevations_rad.append(float(elevation))
        self.in_contact.append(bool(elevation >= min_elevation_rad))

    def elevation_angle(self, position_eci_km, time_s):
        position_ecef = eci_to_ecef(np.asarray(position_eci_km, dtype=float), time_s)
        rho = position_ecef - self.ground_station.position_ecef_km
        rho_norm = np.linalg.norm(rho)
        if rho_norm == 0.0:
            return np.pi / 2.0

        cos_zenith = np.dot(
            rho / rho_norm,
            self.ground_station.local_vertical_ecef,
        )
        return np.arcsin(np.clip(cos_zenith, -1.0, 1.0))

    def _resolve_min_elevation(self, sat):
        if self.min_elevation_rad is not None:
            return self.min_elevation_rad

        if self.antenna_component_name is None:
            raise ValueError("min_elevation_deg is required when no antenna is used.")

        antenna = sat.get_component(self.antenna_component_name)
        self.min_elevation_deg = antenna.min_elevation_deg
        self.min_elevation_rad = np.radians(self.min_elevation_deg)
        return self.min_elevation_rad
