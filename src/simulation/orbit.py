import numpy as np

from ..maths.maths import Omega


class CircularOrbit:
    def __init__(
        self,
        initial_position,
        initial_velocity,
    ):
        self.initial_position = np.asarray(initial_position, dtype=float)
        self.initial_velocity = np.asarray(initial_velocity, dtype=float)

        self.radius_km = np.linalg.norm(self.initial_position)
        self.speed_km_s = np.linalg.norm(self.initial_velocity)
        if self.radius_km == 0.0:
            raise ValueError("initial_position must be non-zero.")

        h = np.cross(self.initial_position, self.initial_velocity)
        h_norm = np.linalg.norm(h)
        if h_norm == 0.0:
            raise ValueError("initial_position and initial_velocity cannot be parallel.")

        self.mean_motion_rad_s = self.speed_km_s / self.radius_km
        self._r_hat = self.initial_position / self.radius_km
        self._h_hat = h / h_norm
        self._v_hat = np.cross(self._h_hat, self._r_hat)

    def position_at(self, time_s):
        theta = self.mean_motion_rad_s * time_s
        return self.radius_km * (
            np.cos(theta) * self._r_hat + np.sin(theta) * self._v_hat
        )

    @classmethod
    def from_satellite(cls, sat):
        return cls(sat.position, sat.velocity)

    def simulate(self, sat, total_time, dt):
        q = np.asarray(sat.quaternion, dtype=float).copy()
        omega = np.asarray(sat.omega, dtype=float).copy()
        steps = int(total_time / dt)

        sat.velocity = self.initial_velocity.copy()
        sat.omega = omega.copy()

        for step in range(steps):
            t = step * dt
            sat.position = self.position_at(t)

            q_dot = 0.5 * Omega(omega) @ q
            q = q + q_dot * dt
            q = q / np.linalg.norm(q)

            sat.quaternion = q.copy()
            sat.time = t

            for monitor in sat.monitors:
                monitor.sample(sat)

        return sat


def simulate_circular_orbit(
    sat,
    total_time,
    dt,
):
    orbit = CircularOrbit.from_satellite(sat)
    return orbit.simulate(sat, total_time, dt)
