import numpy as np

from .base import Component


class Antenna(Component):
    def __init__(
        self,
        min_elevation_deg=30.0,
        boresight_body=np.array([1.0, 0.0, 0.0]),
        name="ttc_antenna",
    ):
        super().__init__(name)
        self.min_elevation_deg = float(min_elevation_deg)
        self.boresight_body = self._unit_vector(boresight_body)

    def _unit_vector(self, value):
        vector = np.asarray(value, dtype=float)
        norm = np.linalg.norm(vector)
        if norm == 0.0:
            raise ValueError("boresight_body must be non-zero.")
        return vector / norm
