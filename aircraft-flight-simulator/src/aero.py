# ─────────────────────────────────────────────────────────────────────────────
# aero.py — Aerodynamic forces and pitching moment
# ─────────────────────────────────────────────────────────────────────────────

import numpy as np
from config import (
    S, C,
    CL0, CL_ALPHA, CL_MAX,
    CD0, K,
    CM0, CM_ALPHA, CM_Q, CM_DELTA_E,
)


def compute_forces(V: float, rho: float, alpha: float,
                   q: float, delta_e: float) -> tuple:
    """
    Compute aerodynamic lift, drag, and pitching moment.

    Parameters
    ----------
    V       : float  — airspeed (m/s)
    rho     : float  — air density (kg/m³)
    alpha   : float  — angle of attack (rad),  clipped to ±15 deg internally
    q       : float  — pitch rate (rad/s)
    delta_e : float  — elevator deflection (rad)

    Returns
    -------
    L, D, M_aero : tuple of floats  — lift (N), drag (N), pitching moment (N·m)
    """
    # Clamp angle of attack to avoid stall model discontinuities
    alpha = np.clip(alpha, np.radians(-15), np.radians(15))

    q_dyn = 0.5 * rho * V**2 * S          # dynamic pressure × wing area

    # Lift
    CL = np.clip(CL0 + CL_ALPHA * alpha, -CL_MAX, CL_MAX)
    L  = q_dyn * CL

    # Drag  (parabolic polar)
    CD = CD0 + K * CL**2
    D  = q_dyn * CD

    # Pitching moment  (includes pitch-rate damping)
    q_hat  = q * C / (2.0 * V)            # normalised pitch rate
    Cm     = CM0 + CM_ALPHA * alpha + CM_Q * q_hat + CM_DELTA_E * delta_e
    M_aero = q_dyn * C * Cm

    return L, D, M_aero