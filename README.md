# 3DoF Longitudinal Flight Dynamics Simulator

A modern C++ three-degree-of-freedom longitudinal flight dynamics simulator.

This project models the motion of an aircraft in the vertical plane using aerodynamic force and moment modelling, an ISA atmosphere approximation, a pitch attitude control law, and fourth-order Runge-Kutta numerical integration. The simulator outputs trajectory data to CSV for post-processing and engineering analysis.



## Project Motivation

This project was developed as part of my transition from aerospace systems engineering into aerospace simulation and flight software engineering. It demonstrates the connection between aerospace engineering theory, numerical methods and modern C++ software implementation.



## Example Results

Example trajectory and response plots are provided in the `/plots` directory, including:



\- Trajectory Profile (Altitude vs Range)

\- Altitude History

\- Velocity History

\- Pitch Response

\- Angle of Attack History

\- Pitching Moment History

\- Lift-to-Drag Ratio History





These plots demonstrate the simulated aircraft response to the implemented aerodynamic model, atmosphere model, 



## Key Features

* 3DoF longitudinal flight dynamics model
* Aircraft state propagation using RK4 integration
* ISA-based atmospheric density model
* Aerodynamic lift, drag and pitching moment calculations
* Pitch attitude control law
* CSV trajectory logging
* Structured C++ codebase using classes and separation of concerns
* C++17 standard library features





## Model States

The simulator propagates the following state vector:

|State|Description|Unit|
|-|-|-|
|`x`|Horizontal range|m|
|`z`|Altitude|m|
|`V`|Airspeed|m/s|
|`gamma`|Flight path angle|rad|
|`theta`|Pitch attitude|rad|
|`q`|Pitch rate|rad/s|

## 

## Equations of Motion


x\_dot     = V cos(gamma)
z\_dot     = V sin(gamma)
V\_dot     = \[T cos(theta - gamma) - D] / m - g sin(gamma)
gamma\_dot = \[T sin(theta - gamma) + L] / (mV) - \[g cos(gamma)] / V
theta\_dot = q
q\_dot     = M / Iyy


## Aerodynamic Model

Dynamic pressure:

q\_bar = 0.5 \* rho \* V^2


Lift coefficient:

CL = CL\_alpha \* alpha + CL\_delta \* delta\_e


Drag coefficient:

CD = CD0 + k \* CL^2


Pitching moment coefficient:

CM = CM\_alpha \* alpha + CM\_q \* (q \* c / 2V) + CM\_delta \* delta\_e


Forces and moment:

L = q\_bar \* S \* CL
D = q\_bar \* S \* CD
M = q\_bar \* S \* c \* CM


## 

## Repository Structure

## Repository Structure

3DoF-Flight-Dynamics-Simulator/
│
├── include/
│   ├── Aero_model.h
│   ├── Atmosphere.h
│   ├── csv_writer.h
│   ├── integrator.h
│   ├── simulator.h
│   ├── State.h
│   ├── vector2.h
│   └── Vehicle.h
│
├── src/
│   ├── Aero_model.cpp
│   ├── Atmosphere.cpp
│   ├── csv_writer.cpp
│   ├── simulator.cpp
│   └── main.cpp
│
├── data/
│   └── trajectory.csv
│
├── docs/
│   ├── Theory.md
│   └── Control_Law.md
│
├── plots/
│   ├── trajectory_profile.png
│   ├── velocity_history.png
│   ├── pitch_response.png
│   ├── alpha_trim.png
│   ├── Lift-to-Drag_Ratio_history.png
│   ├── Pitching_Moment_history.png
│   └── altitude_history.png
│
├── screenshots/
│   ├── build_success.png
│   ├── ide_workspace.png
│   └── runtime_execution.png
│
├── CMakeLists.txt
├── .gitignore
├── LICENSE
└── README.md

## Build Instructions

### Visual Studio

1. Open the project in Visual Studio.
2. Ensure the C++ language standard is set to C++17 or newer.
3. Build the solution.
4. Run the executable.
5. The simulator writes output to:

data/trajectory.csv
C++17 is required because the project uses `std::filesystem`.

### 

### CMake

mkdir build
cd build
cmake ..
cmake --build .


## Output

The simulator generates a CSV trajectory file containing:

* time
* range
* altitude
* velocity
* flight path angle
* pitch angle
* pitch rate
* angle of attack
* lift
* drag
* pitching moment
* air density

The output can be opened directly in Microsoft Excel for plotting and inspection.

## 

## Suggested Manual Plots in Excel

Recommended plots:

* altitude vs range - have
* altitude vs time - have
* velocity vs time - have
* pitch angle vs time - have
* angle of attack vs time - have
* lift and drag vs time - have
* pitch moment vs time

## 

## Engineering Assumptions

This is a simplified longitudinal simulation. The following effects are not currently modelled:

* lateral-directional motion
* roll and yaw dynamics
* wind and turbulence
* actuator dynamics
* sensor models
* propulsion lag
* compressibility effects
* nonlinear aerodynamic lookup tables
* Earth curvature
* rotating reference frames

## 

## Future Improvements

Planned future extensions:

* 6DoF rigid-body dynamics
* quaternion attitude representation
* wind and turbulence modelling
* PID autopilot
* actuator and sensor models
* unit tests
* CI build pipeline using GitHub Actions
* validation against analytical cases or reference data
* optional plotting tools in future

## 

## Author

Aerospace engineer with experience in avionics design, aircraft systems, airworthiness, integration and MRO, developing C++ flight dynamics and simulation projects as part of a transition into aerospace modelling, simulation and flight software engineering.

