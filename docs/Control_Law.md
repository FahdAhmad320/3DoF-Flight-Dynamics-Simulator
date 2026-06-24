# Pitch Attitude Control Law

This document describes the pitch-attitude control architecture implemented within the simulator.

---

# 1. Overview

The aircraft model includes a simplified closed-loop pitch attitude controller.

The objective of the controller is to command elevator deflection in order to regulate aircraft pitch attitude and provide pitch-rate damping.

The controller is intentionally lightweight and is intended to demonstrate control-system integration within a flight dynamics simulation environment.

---

# 2. Control Architecture

The control loop consists of:

Pitch Command
↓
PD Controller
↓
Elevator Deflection
↓
Aircraft Dynamics
↓
Pitch Response

The controller continuously compares the measured pitch attitude with the desired pitch attitude command.

---

# 3. Control Law

Elevator deflection is computed using a proportional-derivative (PD) control law:

δe = Kp(θ − θcmd) + Kd q

Where:

- δe = Elevator Deflection (rad)
- θ = Aircraft Pitch Attitude (rad)
- θcmd = Commanded Pitch Attitude (rad)
- q = Pitch Rate (rad/s)
- Kp = Proportional Gain
- Kd = Derivative Gain

The proportional term provides pitch tracking while the derivative term introduces damping and reduces oscillatory behaviour.

---

# 4. Elevator Limits

The elevator command is constrained using a maximum allowable deflection limit:

|δe| ≤ δe,max

This prevents unrealistic control surface commands and provides a simplified representation of actuator constraints.

---

# 5. Gain Selection

The controller gains were selected empirically to provide stable pitch response within the simulated flight envelope.

The gains are not derived from a formal aircraft linearisation, root-locus analysis or frequency-domain design process.

Instead, they were tuned to demonstrate:

- Stable pitch response
- Reasonable transient behaviour
- Reduced oscillations
- Closed-loop flight-path control

---

# 6. Limitations

The current controller does not model:

- Autopilot logic
- Flight director functionality
- Sensor noise
- Actuator dynamics
- Gain scheduling
- Flight-envelope protection
- Adaptive control

Consequently, the controller should be viewed as a demonstration of control-system integration rather than a production-ready flight-control system.

---

# 7. Future Development

Potential future improvements include:

- PID control
- State-space control
- LQR design
- Gain scheduling
- Autoland logic
- Guidance algorithms
- Full Guidance, Navigation & Control (GNC) architecture