***Aircraft Flight Simulation \& Autopilot Control System***

A nonlinear aircraft flight simulator developed in Python as an independent Aerospace Engineering project.

The simulator models longitudinal aircraft dynamics, aerodynamic forces and moments, aircraft trim conditions, and autopilot control systems for pitch, altitude, and airspeed control.

**Project Structure:**

The 'src' file contains the following python codes needed to run the simulation:



aero.py

atmosphere.py

config.py

controllers.py

trim.py

main.py


The 'plots' file contains screenshots of the simulation running. Two screenshots are given: one where current altitude is 1000m and desired altitude is 1200m, and the other one where current altitude is 1000m and desired altitude is 800m.

**File Descriptions**



1\. aero.py

Computes:
Lift, Drag, Pitching Moments, Aerodynamic Coefficients

2\. atmosphere.py

Provides atmospheric properties including: 
air density,  temperature, pressure

3\. config.py

Stores: 
aircraft parameters, controller gains, simulation settings

4\. controllers.py

Contains:
pitch controller, altitude controller, airspeed controller

5\. trim.py

Calculates equilibrium flight conditions using numerical root finding.

6\. main.py

Runs the simulation, integrates the equations of motion, and generates plots.


**Project Overview:**

This project was developed to apply principles of:

\- Flight Mechanics

\- Aerodynamics

\- Aircraft Stability and Control

\- Numerical Methods



The simulator predicts aircraft motion by solving the nonlinear equations of motion at each time step and updating the aircraft state using numerical integration.

The final model includes:

\- Aerodynamic force and moment modelling

\- Atmospheric modelling

\- Aircraft trim analysis

\- Pitch autopilot

\- Altitude-hold autopilot

\- Airspeed-hold autopilot

\- Data visualisation and response analysis



**Example Of The Simulation Running:**

<img width="1919" height="1079" alt="Altitude_response_1000-1200" src="https://github.com/user-attachments/assets/9b6ba876-e813-4f40-9bdb-28edf06ac990" />



**Aircraft States:**

The longitudinal aircraft model uses the following state variables:



V: Airspeed

γ: Flight Path Angle

q: Pitch Rate

θ: Pitch Angle

h: Altitude

x: Horizontal Position



**Aerodynamic Model:**

Lift Coefficient

CL = CL₀ + CLα α



Drag Coefficient

CD = CD₀ + kCL²



Pitching Moment Coefficient

Cm = Cm₀ + Cmα α + Cmq q̂ + Cmδe δe



Where:

\- α = Angle of Attack

\- δe = Elevator Deflection

\- q̂ = Non-dimensional Pitch Rate



**Aircraft Equations of Motion:**

Airspeed

V̇ = (T cos α − D) / m − g sin γ



Flight Path Angle

γ̇ = (L + T sin α)/(mV) − (g cos γ)/V



Pitch Rate

q̇ = M/Iyy



Pitch Angle

θ̇ = q



Position

ẋ = V cos γ

ḣ = V sin γ



Trim Analysis:

Before the simulation begins, the aircraft trim condition is calculated using SciPy's `fsolve()` routine.

The trim condition is found by solving:

V̇ = 0

γ̇ = 0

q̇ = 0



for:

\- Trim Angle of Attack

\- Trim Elevator Deflection

\- Trim Thrust



This ensures the aircraft starts in steady, level flight before any control inputs are applied.

Trim Residuals In Simulation:



V\_dot     = 6.63e-14

gamma\_dot = 2.78e-17

q\_dot     = 5.14e-17



These values are effectively zero and confirm a valid trim solution.



**Control Architecture:**

The simulator uses a cascaded autopilot structure.



Altitude Controller:

Altitude Error -> Pitch Command



Pitch Controller:

Pitch Error -> PID Controller -> Elevator Deflection



Airspeed Controller:

Airspeed Error -> Throttle Command



**Skills Used:**

\- Python

\- NumPy

\- SciPy

\- Matplotlib



**Learning Outcomes:**

This project provided practical experience in:

\- Flight Dynamics

\- Aerodynamic Modelling

\- Aircraft Stability and Control

\- Trim Analysis

\- PID Controller Design

\- Numerical Simulation

\- Scientific Programming in Python



**Future Improvements**:

Potential future developments include:

\- Full 6-DOF aircraft dynamics

\- State-space linearisation

\- Eigenvalue stability analysis

\- LQR control design

\- Wind disturbance modelling

\- Sensor and actuator dynamics



**Author:**



Altamish Shohed



Queen Mary University of London  

BEng Aerospace Engineering
