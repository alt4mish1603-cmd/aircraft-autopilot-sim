import numpy as np
from scipy.optimize import fsolve

from atmosphere import isa_density
from aero import compute_forces
from config import (GAMMA0, KDH, KIH, KIH, KDH, KPH, V0, H0, DT, M, G, S, C, IYY,
                    CL0, CL_ALPHA, CL_MAX)
import config




def compute_trim(V0, h):
    rho   = isa_density(h)
    q_dyn = 0.5 * rho * V0**2 * config.S

    def equations(vars):
        alpha, delta_e, T = vars

        CL = config.CL0 + config.CL_ALPHA * alpha
        CD = config.CD0 + config.K * CL**2
        Cm = config.CM0 + config.CM_ALPHA * alpha + config.CM_DELTA_E * delta_e
        # Cm_q drops out at trim (q = 0)

        L = q_dyn * CL
        D = q_dyn * CD
        M = q_dyn * config.C * Cm

        # Level flight trim: gamma = 0
        V_dot     = (T * np.cos(alpha) - D) / config.M  - config.G * np.sin(0.0)
        gamma_dot = (L + T * np.sin(alpha)) / (config.M * V0) - config.G * np.cos(0.0) / V0
        q_dot     = M / config.IYY

        return [V_dot, gamma_dot, q_dot]

    CL_guess      = config.M * config.G / q_dyn
    alpha_guess   = (CL_guess - config.CL0) / config.CL_ALPHA
    delta_e_guess = -(config.CM0 + config.CM_ALPHA * alpha_guess) / config.CM_DELTA_E
    T_guess       = q_dyn * (config.CD0 + config.K * CL_guess**2)

    solution, info, ier, msg = fsolve(
        equations,
        [alpha_guess, delta_e_guess, T_guess],
        full_output=True
    )

    if ier != 1:
        print(f"WARNING: Trim did not converge — {msg}")

    alpha_trim, delta_e_trim, T_trim = solution

    residuals = equations(solution)
    print("Trim solution:")
    print(f"  alpha_trim   = {np.degrees(alpha_trim):.6f} deg")
    print(f"  delta_e_trim = {np.degrees(delta_e_trim):.6f} deg")
    print(f"  T_trim       = {T_trim:.6f} N")
    print(f"  rho at h={config.H0} m = {rho:.5f} kg/m³  (ISA, not constant 1.225)")
    print("Trim residuals:")
    print(f"  V_dot     = {residuals[0]:.2e}  (target: 0)")
    print(f"  gamma_dot = {residuals[1]:.2e}  (target: 0)")
    print(f"  q_dot     = {residuals[2]:.2e}  (target: 0)")

    return alpha_trim, delta_e_trim, T_trim


alpha_trim, delta_e_trim, T_trim = compute_trim(V0, config.H0)

print(f"\ndelta_e_trim = {np.degrees(delta_e_trim):.4f} deg  "
      f"({'OK' if abs(np.degrees(delta_e_trim)) <= 30 else 'CLIPPED'} within ±30 deg)")

# ─────────────────────────────────────────────────────────────────────────────
# Warm-start integrators so derivative terms are zero at t = 0
# (avoids the false spike caused by jumping from zero to the real initial error)
# ─────────────────────────────────────────────────────────────────────────────
theta = alpha_trim +  GAMMA0  # gamma = 0 at trim

# Altitude outer loop
alt_error_init     = config.H_COMMAND - config.H0
integral_alt_error = 0.0
alt_previous_error = alt_error_init

theta_command_init = np.clip(
    KPH * alt_error_init + KIH * 0.0 + KDH * 0.0,
    np.radians(-10), np.radians(10)
)

# Pitch inner loop
previous_error = theta_command_init - theta
integral_error = 0.0

# Speed controller
V_command        = V0      # hold trim airspeed (50 m/s)
integral_V_error = 0.0
T                = T_trim  # current thrust — updated each step by throttle controller

# ─────────────────────────────────────────────────────────────────────────────
# Storage arrays
# ─────────────────────────────────────────────────────────────────────────────
time_history      = []
theta_history     = []
theta_cmd_history = []
elevator_history  = []
h_history         = []
h_command_history = []
gamma_history     = []
V_history         = []
T_history         = []
rho_history       = []