# ─────────────────────────────────────────────────────────────────────────────
# atmosphere.py — International Standard Atmosphere (ISA) model
# ─────────────────────────────────────────────────────────────────────────────

import numpy as np
from config import G


def isa_density(altitude: float) -> float:
    """
    Compute air density using the ISA troposphere model (valid 0–11 km).

    At sea level  → 1.225 kg/m³
    At 1000 m     → ~1.112 kg/m³  (constant 1.225 over-estimates by ~10%)
    At 11000 m    → ~0.364 kg/m³

    Parameters
    ----------
    altitude : float  — geometric altitude in metres

    Returns
    -------
    rho : float  — air density in kg/m³
    """
    T0   = 288.15   # K       sea-level temperature
    RHO0 = 1.225    # kg/m³   sea-level density
    L    = 0.0065   # K/m     tropospheric lapse rate
    R    = 287.05   # J/kg·K  specific gas constant for dry air

    alt = np.clip(altitude, 0.0, 11000.0)
    T   = T0 - L * alt
    return RHO0 * (T / T0) ** (G / (R * L) - 1.0)
