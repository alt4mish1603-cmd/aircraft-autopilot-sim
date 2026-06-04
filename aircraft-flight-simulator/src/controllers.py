# ─────────────────────────────────────────────────────────────────────────────
# controllers.py — Cascaded PID autopilot (altitude → pitch → elevator + throttle)
# ─────────────────────────────────────────────────────────────────────────────

import numpy as np
from config import KPH, KIH, KDH, KP, KI, KD, KP_T, KI_T


class AltitudeController:
    """
    Outer loop: altitude error → theta_command.

    Proportional on altitude error, derivative provides anticipatory
    damping as the aircraft approaches the commanded altitude.
    Output is clipped to ±10 deg to prevent aggressive pitch commands.
    """

    def __init__(self):
        self.integral    = 0.0
        self.prev_error  = None          # set on first call via warm_start()

    def warm_start(self, h: float, h_command: float):
        """
        Initialise prev_error to the real t=0 error so the derivative
        term is zero on the first timestep (avoids a false spike).
        """
        self.prev_error = h_command - h

    def update(self, h: float, h_command: float, dt: float) -> float:
        """
        Parameters
        ----------
        h         : current altitude (m)
        h_command : commanded altitude (m)
        dt        : timestep (s)

        Returns
        -------
        theta_command : float  — commanded pitch angle (rad)
        """
        error = h_command - h

        self.integral = np.clip(
            self.integral + error * dt,
            np.radians(-10), np.radians(10)
        )

        if self.prev_error is None:
            self.prev_error = error

        derivative      = (error - self.prev_error) / dt
        self.prev_error = error

        theta_command = (
            KPH * error
            + KIH * self.integral
            + KDH * derivative
        )

        return np.clip(theta_command, np.radians(-10), np.radians(10))


class PitchController:
    """
    Inner loop: pitch error → elevator deflection (delta_e).

    Negative gains because a positive pitch error (theta < theta_command)
    requires a nose-up elevator deflection (negative delta_e convention).
    Output is added to delta_e_trim and clipped to ±30 deg.
    """

    def __init__(self, delta_e_trim: float):
        self.delta_e_trim = delta_e_trim
        self.integral     = 0.0
        self.prev_error   = None         # set via warm_start()

    def warm_start(self, theta: float, theta_command_init: float):
        """
        Initialise prev_error to the real t=0 pitch error so the
        derivative term is zero on the first timestep.
        """
        self.prev_error = theta_command_init - theta

    def update(self, theta: float, theta_command: float, dt: float) -> float:
        """
        Parameters
        ----------
        theta         : current pitch angle (rad)
        theta_command : commanded pitch angle (rad)
        dt            : timestep (s)

        Returns
        -------
        delta_e : float  — elevator deflection (rad)
        """
        error = theta_command - theta

        self.integral = np.clip(
            self.integral + error * dt,
            np.radians(-20), np.radians(20)
        )

        if self.prev_error is None:
            self.prev_error = error

        derivative      = (error - self.prev_error) / dt
        self.prev_error = error

        delta_e = (
            self.delta_e_trim
            + KP * error
            + KI * self.integral
            + KD * derivative
        )

        return np.clip(delta_e, np.radians(-30), np.radians(30))


class ThrottleController:
    """
    Speed hold: airspeed error → thrust.

    Without this, thrust is frozen at T_trim while the gravity component
    during climb bleeds speed and robs the pitch loop of authority.
    Thrust is capped at 2.5 × T_trim to stay physically plausible.
    """

    def __init__(self, T_trim: float, V_command: float):
        self.T_trim    = T_trim
        self.V_command = V_command
        self.integral  = 0.0

    def update(self, V: float, dt: float) -> float:
        """
        Parameters
        ----------
        V  : current airspeed (m/s)
        dt : timestep (s)

        Returns
        -------
        T : float  — thrust (N)
        """
        error          = self.V_command - V
        self.integral += error * dt

        T = (
            self.T_trim
            + KP_T * error
            + KI_T * self.integral
        )

        return np.clip(T, 0.0, 2.5 * self.T_trim)