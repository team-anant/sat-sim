import unittest

import numpy as np

from src import Satellite
from src.analysis.attitude import BodyAxisAlignmentMonitor
from src.analysis.monitor import AnalysisMonitor
from src.analysis.solarpower import SolarPowerMonitor
from src.analysis.ttc import TtcContactMonitor
from src.ground import GroundStation
from src.maths.maths import eci_to_ecef
from src.satellite.components.power import SolarPanelArray
from src.satellite.components.surfaces import BodySurfaceModel
from src.satellite.components.ttc import Antenna
from src.simulation.orbit import CircularOrbit, simulate_circular_orbit
from src.utils.initial_conditions import generate_inclined_circular_orbit


class DummyMonitor(AnalysisMonitor):
    def __init__(self):
        super().__init__("dummy")
        self.attached_sat = None
        self.sample_times = []

    def attach(self, sat):
        self.attached_sat = sat

    def sample(self, sat):
        self.sample_times.append(sat.time)


def satellite_at(position):
    sat = Satellite()
    sat.position = np.asarray(position, dtype=float)
    sat.velocity = np.array([0.0, 7.67, 0.0])
    sat.quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    sat.omega = np.zeros(3)
    return sat


class AnalysisMonitorTests(unittest.TestCase):
    def test_satellite_has_default_hardware_components(self):
        sat = Satellite()

        self.assertIsInstance(sat.get_component("body_surfaces"), BodySurfaceModel)
        self.assertIsInstance(sat.get_component("solar_panels"), SolarPanelArray)
        self.assertEqual(sat.n.shape, (6, 3))
        self.assertEqual(sat.A.shape, (6,))
        self.assertEqual(sat.solarp_area.shape, (6,))
        self.assertEqual(len(sat.get_components(SolarPanelArray)), 1)

    def test_satellite_rejects_duplicate_component_names(self):
        sat = Satellite()

        with self.assertRaises(ValueError):
            sat.add_component(BodySurfaceModel())

    def test_satellite_samples_attached_monitor_after_propagate(self):
        sat = satellite_at([6778.0, 0.0, 0.0])
        monitor = sat.add_monitor(DummyMonitor())

        sat.propagate(1.0)

        self.assertIs(monitor.attached_sat, sat)
        self.assertEqual(len(monitor.sample_times), 1)
        self.assertEqual(monitor.sample_times[0], sat.time)

    def test_solar_power_is_positive_when_panel_faces_sun(self):
        sat = satellite_at([0.0, 6778.0, 0.0])
        monitor = SolarPowerMonitor(
            sun_position_km=np.array([1.496e8, 0.0, 0.0]),
            solar_panel_areas=np.ones(6),
        )

        monitor.sample(sat)

        self.assertFalse(monitor.in_eclipse[-1])
        self.assertGreater(monitor.powers[-1], 0.0)

    def test_solar_power_monitor_uses_solar_panel_component(self):
        sat = satellite_at([0.0, 6778.0, 0.0])
        sat.get_component("solar_panels").areas_m2 = np.ones(6)
        monitor = SolarPowerMonitor(sun_position_km=np.array([1.496e8, 0.0, 0.0]))

        monitor.sample(sat)

        self.assertGreater(monitor.powers[-1], 0.0)

    def test_solar_power_is_zero_in_eclipse(self):
        sat = satellite_at([-6778.0, 0.0, 0.0])
        monitor = SolarPowerMonitor(
            sun_position_km=np.array([1.496e8, 0.0, 0.0]),
            solar_panel_areas=np.ones(6),
        )

        monitor.sample(sat)

        self.assertTrue(monitor.in_eclipse[-1])
        self.assertEqual(monitor.powers[-1], 0.0)

    def test_solar_monitor_histories_have_matching_lengths(self):
        sat = satellite_at([6778.0, 0.0, 0.0])
        monitor = sat.add_monitor(SolarPowerMonitor())

        for _ in range(3):
            sat.propagate(1.0)

        self.assertEqual(len(monitor.times), 3)
        self.assertEqual(len(monitor.powers), 3)
        self.assertEqual(len(monitor.in_eclipse), 3)
        self.assertEqual(len(monitor.sun_dirs_body), 3)

    def test_circular_orbit_simulation_samples_arbitrary_monitors(self):
        sat = satellite_at([6778.0, 0.0, 0.0])
        dummy = sat.add_monitor(DummyMonitor())
        alignment = sat.add_monitor(BodyAxisAlignmentMonitor())

        simulate_circular_orbit(
            sat,
            total_time=30.0,
            dt=10.0,
        )

        self.assertEqual(len(dummy.sample_times), 3)
        self.assertEqual(len(alignment.alignments), 3)

    def test_circular_orbit_class_samples_arbitrary_monitors(self):
        sat = satellite_at([6778.0, 0.0, 0.0])
        dummy = sat.add_monitor(DummyMonitor())
        orbit = CircularOrbit.from_satellite(sat)

        orbit.simulate(sat, total_time=20.0, dt=10.0)

        self.assertEqual(len(dummy.sample_times), 2)

    def test_circular_orbit_uses_satellite_attitude_state(self):
        sat = satellite_at([6778.0, 0.0, 0.0])
        sat.omega = np.array([0.0, 0.0, 0.1])
        orbit = CircularOrbit.from_satellite(sat)

        orbit.simulate(sat, total_time=20.0, dt=10.0)

        self.assertFalse(np.allclose(sat.quaternion, np.array([1.0, 0.0, 0.0, 0.0])))

    def test_generate_inclined_circular_orbit_returns_perpendicular_state(self):
        r, v = generate_inclined_circular_orbit(
            inclination_deg=60.0,
            radius_km=6771.0,
            raan_deg=10.0,
            arg_lat_deg=25.0,
        )

        self.assertAlmostEqual(np.linalg.norm(r), 6771.0)
        self.assertAlmostEqual(np.dot(r, v), 0.0, places=8)

    def test_ttc_contact_monitor_detects_overhead_contact(self):
        sat = satellite_at([6878.0, 0.0, 0.0])
        ground_station = GroundStation(latitude_deg=0.0, longitude_deg=0.0)
        monitor = TtcContactMonitor(
            ground_station=ground_station,
            min_elevation_deg=30.0,
            sample_period_s=1.0,
        )

        monitor.sample(sat)

        self.assertTrue(monitor.in_contact[-1])
        self.assertGreater(monitor.total_contact_time_s, 0.0)

    def test_eci_to_ecef_is_identity_at_zero_time(self):
        position = np.array([1.0, 2.0, 3.0])

        np.testing.assert_allclose(eci_to_ecef(position, 0.0), position)

    def test_ttc_contact_monitor_rejects_below_horizon_contact(self):
        sat = satellite_at([-6878.0, 0.0, 0.0])
        ground_station = GroundStation(latitude_deg=0.0, longitude_deg=0.0)
        monitor = TtcContactMonitor(
            ground_station=ground_station,
            min_elevation_deg=30.0,
            sample_period_s=1.0,
        )

        monitor.sample(sat)

        self.assertFalse(monitor.in_contact[-1])
        self.assertEqual(monitor.total_contact_time_s, 0.0)

    def test_ttc_contact_monitor_can_use_antenna_component_threshold(self):
        sat = satellite_at([6878.0, 0.0, 0.0])
        sat.add_component(Antenna(min_elevation_deg=30.0))
        ground_station = GroundStation(latitude_deg=0.0, longitude_deg=0.0)
        monitor = sat.add_monitor(
            TtcContactMonitor(ground_station=ground_station, sample_period_s=1.0)
        )

        monitor.sample(sat)

        self.assertEqual(monitor.min_elevation_deg, 30.0)
        self.assertTrue(monitor.in_contact[-1])


if __name__ == "__main__":
    unittest.main()
