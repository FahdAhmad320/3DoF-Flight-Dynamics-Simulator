# Flight Mechanics, Atmospheric Physics \& Numerical Integration

This document describes the aerospace engineering theory, mathematical models and numerical methods implemented within the 3DoF longitudinal flight dynamics simulator.

\---

# 1\. Longitudinal 3DoF State-Space Model

The simulator models aircraft motion in the vertical longitudinal plane using a simplified three-degree-of-freedom (3DoF) rigid-body representation.

The state vector is:

x = \[x, z, V, γ, θ, q]ᵀ

Where:

* x = Horizontal position (m)
* z = Altitude (m)
* V = True Airspeed (m/s)
* γ = Flight Path Angle (rad)
* θ = Pitch Attitude Angle (rad)
* q = Pitch Rate (rad/s)

This representation captures longitudinal translational and rotational motion while neglecting lateral-directional dynamics.

\---

# 2\. Equations of Motion

The governing differential equations implemented within the simulator are:

ẋ = V cos(γ)

ż = V sin(γ)

V̇ = \[T cos(θ − γ) − D] / m − g sin(γ)

γ̇ = \[T sin(θ − γ) + L] / (mV) − \[g cos(γ)] / V

θ̇ = q

q̇ = M / Iyy

Where:

* T = Thrust (N)
* L = Lift (N)
* D = Drag (N)
* M = Pitching Moment (Nm)
* m = Vehicle Mass (kg)
* Iyy = Pitch Moment of Inertia (kg·m²)
* g = Gravitational Acceleration (9.81 m/s²)

\---

# 3\. Aerodynamic Force and Moment Model

Dynamic pressure is calculated using:

q∞ = 0.5ρV²

The aerodynamic coefficients are represented using a linearised model:

CL = CLα α + CLδ δe

CD = CD0 + KCL²

Cm = Cmα α + Cmq(qc̄/2V) + Cmδ δe

Where:

* α = Angle of Attack
* δe = Elevator Deflection
* c̄ = Reference Chord

Aerodynamic forces and moments are then computed as:

L = q∞ S CL

D = q∞ S CD

M = q∞ S c̄ Cm

The angle of attack is determined from:

α = θ − γ

\---

# 4\. International Standard Atmosphere (ISA)

Atmospheric density is determined from altitude using a simplified ISA model.

Troposphere (0–11 km):

T(z) = T₀ + Lz

P(z) = P₀ (1 + Lz/T₀)^(-g/RL)

Density is obtained from the Ideal Gas Law:

ρ = P / RT

Where:

* T₀ = 288.15 K
* P₀ = 101325 Pa
* R = 287.05 J/(kg·K)

This enables aerodynamic force variation with altitude.

\---

# 5\. Numerical Integration

State propagation is performed using a fixed-step Fourth Order Runge-Kutta (RK4) integration scheme.

The RK4 algorithm evaluates four intermediate derivative estimates:

k₁
k₂
k₃
k₄

and updates the state using:

xₙ₊₁ = xₙ + Δt/6 (k₁ + 2k₂ + 2k₃ + k₄)

RK4 was selected due to its balance of computational efficiency and numerical accuracy for engineering simulations.

The implementation uses lightweight state and derivative structures, avoiding unnecessary dynamic allocation inside the integration step.

\---

# 6\. Model Assumptions

The current simulator is intentionally simplified and does not include:

* Roll dynamics
* Yaw dynamics
* Wind or turbulence
* Actuator dynamics
* Sensor models
* Ground effect
* Landing gear dynamics
* Propulsion lag
* Non-linear aerodynamic lookup tables

The objective is to demonstrate flight dynamics modelling, numerical integration and aerospace software architecture rather than provide a validated aircraft model.

\---

# 7\. Future Development

Potential future enhancements include:

* Full 6DoF rigid-body dynamics
* PID or autopilot architectures
* Sensor and actuator modelling
* Monte Carlo simulation capability

