import numpy as np

from ..constants import EARTH_RADIUS


class GroundStation:
    def __init__(
        self,
        latitude_deg: float,
        longitude_deg: float,
        radius_km: float = EARTH_RADIUS,
    ):
        self.latitude_deg = float(latitude_deg)
        self.longitude_deg = float(longitude_deg)
        self.radius_km = float(radius_km)

        lat = np.radians(self.latitude_deg)
        lon = np.radians(self.longitude_deg)
        self.position_ecef_km = self.radius_km * np.array(
            [
                np.cos(lat) * np.cos(lon),
                np.cos(lat) * np.sin(lon),
                np.sin(lat),
            ]
        )
        self.local_vertical_ecef = self.position_ecef_km / np.linalg.norm(
            self.position_ecef_km
        )
