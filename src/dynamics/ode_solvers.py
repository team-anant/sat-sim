import numpy as np

from ..constants import EARTH_MU
from .atmospheric_drag import acc_drag
from .j2_acceleration import acc_j2
from .solar_radiation_force import solar_radiation_force
from ..maths.maths import Omega


def position_ode(t, state, sat, quaternion=None):
    r = state[:3]
    r_norm = np.linalg.norm(r)
    if r_norm == 0.0:
        raise ValueError("Position state must be non-zero before propagation.")

    a_kepler = -EARTH_MU * r / r_norm**3
    a_j2 = acc_j2(r)
    # a_solar_radiation = solar_radiation_force(r, sat, t)
    a_drag = acc_drag(r, state[3:6], sat, quaternion=quaternion)

    a = a_kepler + a_j2 + a_drag
    return np.concatenate((state[3:6], a))


def attitude_ode(t, state, sat):
    q = state[:4]
    w = state[4:]
    q_dot = 0.5 * Omega(w).dot(q)

    # TODO External Torques (Gravity Gradient, Atmospheric, Solar Radiation)
    Torque = np.zeros(3)

    w_dot = sat.J_inv.dot(Torque - np.cross(w, np.dot(sat.J, w)))
    return np.concatenate([q_dot, w_dot])


def rk4_step(f, t, y, dt):
    k1 = f(t, y)
    k2 = f(t + 0.5 * dt, y + 0.5 * k1 * dt)
    k3 = f(t + 0.5 * dt, y + 0.5 * k2 * dt)
    k4 = f(t + dt, y + k3 * dt)
    return y + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6


def combined_ode(t, state, sat):
    pos_state = state[:6]
    q = state[6:10]
    w = state[10:]

    pos_dot = position_ode(t, pos_state, sat, quaternion=q)
    att_dot = attitude_ode(t, np.concatenate((q, w)), sat)
    return np.concatenate((pos_dot, att_dot))


def combined_rk4_step(t, sat, dt):
    y0 = np.concatenate((sat.position, sat.velocity, sat.quaternion, sat.omega))
    y = rk4_step(lambda t_, y_: combined_ode(t_, y_, sat), t, y0, dt)
    y[6:10] /= np.linalg.norm(y[6:10])
    return y


def position_rk4_step(t, sat, dt):
    f = lambda t_, y_: position_ode(t_, y_, sat)
    y = np.concatenate((sat.position, sat.velocity))
    return rk4_step(f, t, y, dt)


def attitude_rk4_step(t, sat, dt):
    f = lambda t_, y_: attitude_ode(t_, y_, sat)
    y = rk4_step(f, t, np.concatenate((sat.quaternion, sat.omega)), dt)
    y[:4] /= np.linalg.norm(y[:4])
    return y
