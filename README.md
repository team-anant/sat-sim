# orbit-propagator-3d

Orbit propagation and spacecraft analysis sandbox for simulating satellite state, attachable spacecraft hardware, and monitor-based analyses such as solar power and TTC contact.

## Conventions

- Distances use `km` unless a name/comment explicitly says otherwise, such as `areas_m2`.
- Velocities use `km/s`.
- Accelerations use `km/s^2`.
- Quaternions are ordered `w, x, y, z`.
- Position and velocity are ECI-frame values.
- Satellite quaternion represents ECI to body rotation.
- Satellite angular velocity `omega` is body-frame `rad/s`.

## Running Scripts

Run commands from the repository root.

```powershell
python scripts/main.py
```

Propagates the orbit in `config.json`, samples attached monitors, prints periodic solar-power status, and opens a Plotly trajectory/power visualisation.

```powershell
python scripts/montecarlo_power.py
```

Runs a Monte Carlo solar-power analysis over random inclined circular orbits and random initial attitudes. It uses `SolarPowerMonitor` and `BodyAxisAlignmentMonitor`.

```powershell
python scripts/montecarlo_gps.py
```

Runs a Monte Carlo TTC contact analysis for a ground station over sampled RAAN values. It uses `TtcContactMonitor`, `GroundStation`, and an `Antenna` component.

The GPS/TTC script defaults to a fine `0.1 s` timestep and `1000` samples, so it can be slow. For quick checks, import the script and override `N_SAMPLES` / `DT`.

## Verification

```powershell
python -m unittest discover -s tests
python -m compileall .\src .\tests .\scripts
python -m py_compile .\scripts\main.py .\scripts\montecarlo_power.py .\scripts\montecarlo_gps.py
```

## Design Decisions

The project is organized around a few core choices:

- `Satellite` is the source of truth for spacecraft state. Position, velocity, attitude, angular velocity, mass, inertia, time, components, and monitors live on the satellite.
- Satellite hardware is modeled as pluggable components. Solar panels, antennas, surface geometry, and future reaction wheels or batteries should be attached to `Satellite`, not hardcoded into the satellite class.
- Analyses are modeled as monitors. A monitor observes satellite state after each propagation/simulation step and records histories such as power, contact, attitude alignment, or future battery/data metrics.
- Dynamics query components. Force and torque models should ask the satellite what hardware it has instead of assuming every satellite has the same surfaces, wheels, thrusters, or panels.
- Simulation models advance state but do not decide what to analyze. Orbit models should update `Satellite`; monitors decide what values to record.
- Generic math stays generic. Coordinate transforms and vector/quaternion helpers belong in `src/maths`, not in analysis modules.
- Ground-side objects are separate from spacecraft hardware. A ground station is not a satellite component, so it lives under `src/ground`.
- Scripts are experiments, not libraries. Scripts should configure and run scenarios, while reusable physics, geometry, components, and analyses live in `src`.
- Units should be visible from names/comments whenever they are not the project default. For example, orbit distances are `km`, but component areas are named `areas_m2`.

## Architecture

The project separates spacecraft state, spacecraft hardware, simulation models, force models, and analysis monitors.

### Satellite State

`src/satellite/satellite.py` owns the canonical spacecraft state:

- `position`
- `velocity`
- `quaternion`
- `omega`
- `mass`
- `J`, `J_inv`
- `time`
- attached `components`
- attached `monitors`

`Satellite` should remain the state owner. Simulation code should read and update state on `sat`; it should not carry separate attitude/velocity copies unless there is a specific numerical reason.

### Satellite Components

Satellite-specific hardware belongs in `src/satellite/components/`.

Current components:

- `BodySurfaceModel`: body-frame face normals, drag areas, drag coefficient.
- `SolarPanelArray`: solar-panel areas and efficiency.
- `Antenna`: TTC antenna configuration, currently min elevation and boresight.

Use components for things the spacecraft physically has:

- solar panels
- antennas
- batteries
- reaction wheels
- thrusters
- magnetorquers
- payloads
- onboard data storage

Attach components with:

```python
sat.add_component(Antenna(min_elevation_deg=30.0))
antenna = sat.get_component("ttc_antenna")
panels = sat.get_components(SolarPanelArray)
```

`Satellite` currently installs default `BodySurfaceModel` and `SolarPanelArray` for compatibility with existing analyses.

### Analysis Monitors

Analysis monitors belong in `src/analysis/`.

Current monitors:

- `SolarPowerMonitor`: samples generated solar power and eclipse state.
- `TtcContactMonitor`: samples ground-station contact/elevation.
- `BodyAxisAlignmentMonitor`: samples body-axis alignment against an ECI reference axis.

Monitors observe the satellite. They should not own propagation logic or define generic geometry classes.

Use monitors for things we want to record:

- generated solar power
- eclipse history
- TTC contact time
- link margin
- downlinked data
- battery state history
- reaction wheel momentum history

Attach monitors with:

```python
power = sat.add_monitor(SolarPowerMonitor())
contact = sat.add_monitor(TtcContactMonitor(ground_station=gs))
```

During `sat.propagate(dt)` or `CircularOrbit.simulate(...)`, monitors are sampled after each state update.

### Simulation Models

Simulation models belong in `src/simulation/`.

Current model:

- `CircularOrbit`: closed-form circular orbit state generator for fast experiments.

`CircularOrbit` uses `sat.position` and `sat.velocity` to define the orbit, and uses `sat.quaternion` and `sat.omega` for attitude propagation.

Preferred pattern:

```python
sat.position = r0
sat.velocity = v0
sat.quaternion = q0
sat.omega = omega0

orbit = CircularOrbit.from_satellite(sat)
orbit.simulate(sat, total_time, dt)
```

Add new orbit/simulation approximations here, not in scripts.

### Dynamics

Numerical force/torque models belong in `src/dynamics/`.

Current dynamics include:

- atmospheric drag
- J2 acceleration
- RK4 ODE stepping
- placeholder solar radiation pressure

Dynamics should query satellite components for hardware-dependent effects. For example, drag uses `BodySurfaceModel` instead of hardcoded `sat.A` / `sat.n`.

Future examples:

- reaction wheel torque should query reaction wheel components
- SRP should query surface/solar-panel components
- thrust should query thruster components
- magnetorquer torque should query magnetorquer components and environment data

### Math And Geometry

Generic math belongs in `src/maths/`.

Current examples:

- quaternion/DCM helpers
- angular velocity matrix `Omega`
- vector-angle helpers
- `eci_to_ecef`

Frame conversion and reusable geometry should go here, unless it represents a real-world object.

### Ground Objects

Ground-side, non-satellite objects belong in `src/ground/`.

Current object:

- `GroundStation`: latitude, longitude, ECEF position, local vertical.

Ground stations are not satellite hardware and should not live in `src/satellite/components` or `src/analysis`.

### Utility Helpers

Experiment setup helpers belong in `src/utils/`.

Current helpers:

- `generate_inclined_circular_orbit`
- `random_quaternion`

Use this area for reusable initial-condition generation or other small setup helpers. If a helper becomes a physical model, move it to `simulation`, `dynamics`, `ground`, or `satellite/components`.

### Scripts

Scripts belong in `scripts/`.

Scripts should be thin orchestration layers:

- configure constants
- generate initial conditions
- create a `Satellite`
- attach components
- attach monitors
- run a simulation/propagation model
- print and plot results

Scripts should not define reusable physics, coordinate conversion, hardware models, or analysis logic. Move those into `src`.

## Where To Add New Things

Add a new spacecraft hardware item:

- Put it in `src/satellite/components/`.
- Inherit from `Component`.
- Give it a stable `name`.
- Add validation in `__init__` or `attach`.

Add a new analysis:

- Put it in `src/analysis/`.
- Inherit from `AnalysisMonitor`.
- Implement `sample(self, sat)`.
- Store histories as arrays/lists on the monitor.

Add a new force/torque model:

- Put it in `src/dynamics/`.
- Query satellite components for geometry/hardware.
- Keep units explicit in names/comments.

Add a new orbit/simulation shortcut:

- Put it in `src/simulation/`.
- Let `Satellite` remain the state owner.
- Sample `sat.monitors` after state updates.

Add a new initial-condition generator:

- Put it in `src/utils/initial_conditions.py` unless it grows into a full simulation model.

Add ground infrastructure:

- Put ground stations and ground-side objects in `src/ground/`.

Add a new experiment:

- Put it in `scripts/`.
- Import reusable logic from `src`.
- Avoid adding reusable physics/math inside the script.
