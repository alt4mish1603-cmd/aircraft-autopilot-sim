# ─────────────────────────────────────────────────────────────────────────────
# main.py — Simulation loop and plotting
# Run this file to execute the full simulation.
# ─────────────────────────────────────────────────────────────────────────────

import numpy as np
import matplotlib.pyplot as plt

from config import (
    G, M, IYY,
    H_COMMAND, DT, T_FINAL,
    X0, H0, V0, GAMMA0, Q0,
)
from atmosphere  import isa_density
from aero        import compute_forces
from trim        import compute_trim
from controllers import AltitudeController, PitchController, ThrottleController


# ─────────────────────────────────────────────────────────────────────────────
# 1. Trim
# ─────────────────────────────────────────────────────────────────────────────
alpha_trim, delta_e_trim, T_trim = compute_trim(V0, H0)

# ─────────────────────────────────────────────────────────────────────────────
# 2. Initialise state
# ─────────────────────────────────────────────────────────────────────────────
x     = X0
h     = H0
V     = V0
gamma = GAMMA0
q     = Q0
theta = alpha_trim + gamma   # level flight at trim: theta = alpha (gamma = 0)

# ─────────────────────────────────────────────────────────────────────────────
# 3. Initialise controllers with warm-started integrators
#    (prevents false derivative spike on the first timestep)
# ─────────────────────────────────────────────────────────────────────────────
alt_ctrl   = AltitudeController()
alt_ctrl.warm_start(h, H_COMMAND)

# Compute the t=0 theta_command so the pitch controller can warm-start too
theta_command_init = alt_ctrl.update(h, H_COMMAND, DT)

pitch_ctrl = PitchController(delta_e_trim)
pitch_ctrl.warm_start(theta, theta_command_init)

# Re-initialise alt_ctrl so the first real loop iteration starts fresh
alt_ctrl   = AltitudeController()
alt_ctrl.warm_start(h, H_COMMAND)

throttle_ctrl = ThrottleController(T_trim, V_command=V0)

# ─────────────────────────────────────────────────────────────────────────────
# 4. Storage
# ─────────────────────────────────────────────────────────────────────────────
time_history      = []
theta_history     = []
theta_cmd_history = []
elevator_history  = []
h_history         = []
gamma_history     = []
V_history         = []
T_history         = []

# ─────────────────────────────────────────────────────────────────────────────
# 5. Simulation loop
# ─────────────────────────────────────────────────────────────────────────────
t = 0.0

while t < T_FINAL:
    V = max(V, 1.0)   # prevent divide-by-zero at near-zero speed

    # ── Atmosphere ────────────────────────────────────────────────────────────
    rho = isa_density(h)

    # ── Controllers ───────────────────────────────────────────────────────────
    theta_command = alt_ctrl.update(h, H_COMMAND, DT)
    delta_e       = pitch_ctrl.update(theta, theta_command, DT)
    T             = throttle_ctrl.update(V, DT)

    # ── Aerodynamics ──────────────────────────────────────────────────────────
    alpha       = np.clip(theta - gamma, np.radians(-15), np.radians(15))
    L, D, M_aero = compute_forces(V, rho, alpha, q, delta_e)

    # ── Equations of motion ───────────────────────────────────────────────────
    V_dot     = (T * np.cos(alpha) - D) / M - G * np.sin(gamma)
    gamma_dot = (L + T * np.sin(alpha)) / (M * V) - G * np.cos(gamma) / V
    q_dot     = M_aero / IYY
    theta_dot = q
    h_dot     = V * np.sin(gamma)
    x_dot     = V * np.cos(gamma)

    # ── Euler integration ─────────────────────────────────────────────────────
    V     += V_dot     * DT
    gamma += gamma_dot * DT
    q     += q_dot     * DT
    theta += theta_dot * DT
    h     += h_dot     * DT
    x     += x_dot     * DT

    # ── Store ─────────────────────────────────────────────────────────────────
    time_history.append(t)
    theta_history.append(np.degrees(theta))
    theta_cmd_history.append(np.degrees(theta_command))
    elevator_history.append(np.degrees(delta_e))
    h_history.append(h)
    gamma_history.append(np.degrees(gamma))
    V_history.append(V)
    T_history.append(T)

    t += DT

# ─────────────────────────────────────────────────────────────────────────────
# 6. Plots
# ─────────────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(5, 1, figsize=(11, 18), sharex=True)
fig.suptitle("Aircraft Altitude Hold Simulation", fontsize=13, y=0.998)

axes[0].plot(time_history, theta_history,     label="Pitch Angle θ")
axes[0].plot(time_history, theta_cmd_history, "--", label="Pitch Command")
axes[0].set_ylabel("Pitch (deg)")
axes[0].legend(); axes[0].grid(True)

axes[1].plot(time_history, elevator_history, color="tab:orange")
axes[1].axhline(np.degrees(delta_e_trim), color="k", linestyle=":",
                linewidth=0.8, label=f"δe_trim = {np.degrees(delta_e_trim):.2f}°")
axes[1].set_ylabel("Elevator (deg)")
axes[1].legend(); axes[1].grid(True)

axes[2].plot(time_history, h_history, label="Altitude")
axes[2].axhline(H_COMMAND, color="k", linestyle="--",
                linewidth=0.8, label=f"Command ({H_COMMAND} m)")
axes[2].set_ylabel("Altitude (m)")
axes[2].legend(); axes[2].grid(True)

axes[3].plot(time_history, V_history, color="tab:green", label="Airspeed V")
axes[3].axhline(V0, color="k", linestyle="--",
                linewidth=0.8, label=f"V_cmd = {V0} m/s")
axes[3].set_ylabel("Airspeed (m/s)")
axes[3].legend(); axes[3].grid(True)

axes[4].plot(time_history, T_history, color="tab:red", label="Thrust T")
axes[4].axhline(T_trim, color="k", linestyle="--",
                linewidth=0.8, label=f"T_trim = {T_trim:.1f} N")
axes[4].set_ylabel("Thrust (N)")
axes[4].set_xlabel("Time (s)")
axes[4].legend(); axes[4].grid(True)

plt.tight_layout()
plt.savefig("plots/altitude_hold.png", dpi=150)
plt.show()