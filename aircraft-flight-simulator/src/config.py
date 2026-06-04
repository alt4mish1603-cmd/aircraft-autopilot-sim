# ─────────────────────────────────────────────────────────────────────────────
# config.py — All aircraft parameters, aero coefficients, and controller gains
# Change values here; everything else reads from this file.
# ─────────────────────────────────────────────────────────────────────────────

# Physical constants
G   = 9.81      # m/s²

# Aircraft parameters
M   = 1200.0    # kg
IYY = 4500.0    # kg·m²  (pitch moment of inertia)
S   = 16.0      # m²     (wing area)
C   = 1.5       # m      (mean aerodynamic chord)

# Aerodynamic coefficients
CL0      =  0.2
CL_ALPHA =  5.5
CL_MAX   =  1.5

CD0 =  0.02
K   =  0.04

CM0        =  0.0
CM_ALPHA   = -0.8
CM_Q       = -14.0
CM_DELTA_E = -1.2

# ── Controller Gains ──────────────────────────────────────────────────────────

# Altitude hold (outer PID loop)
H_COMMAND = 2000.0   # m   — commanded altitude

KPH =  0.005
KIH =  0.0
KDH =  0.0005   # positive → correct anticipatory damping on approach

# Pitch hold (inner PID loop)
KP = -1.0
KI = -0.05   # small integral removes steady-state pitch error
KD = -0.8

# Speed hold (throttle PI)
KP_T = 300.0    # N / (m/s)
KI_T =  30.0    # N / (m/s²)

# ── Simulation Settings ───────────────────────────────────────────────────────
DT      = 0.01
T_FINAL = 2000.0

# ── Initial Conditions ────────────────────────────────────────────────────────
X0     = 0.0
H0     = 1000.0   # m
V0     = 50.0     # m/s
GAMMA0 = 0.0      # rad
Q0     = 0.0      # rad/s