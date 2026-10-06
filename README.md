# Non-linear-Hinfinity-with-observer-version1


# Nonlinear H∞ Output-Feedback Control for Semi-Active Suspension of an In-Wheel-Motor Vehicle

Bachelor's thesis project, Chiba institute of technology (2022). MATLAB/Simulink.

## Overview
In-wheel-motor (IWM) vehicles place the motor inside the wheel, which increases unsprung mass and degrades ride comfort and road holding. This project designs a nonlinear H∞ output-feedback controller for a semi-active suspension to suppress vertical vibration in a quarter-car model.

## Why a bilinear system
A semi-active damper can only change its damping coefficient. The damper force is the product of the control input (the damping coefficient) and a state (the relative velocity), so the system is bilinear, not linear. I applied the nonlinear H∞ control theory for bilinear systems proposed by Etsuro Shimizu from Tokyo University of Marine Science and Technology (1999).

## Method
- **Model:** quarter-car model with an in-wheel motor ([sprung mass, unsprung mass, tire stiffness, ...])
- **Controller:** nonlinear H∞ state feedback for bilinear systems
- **Observer:** estimates unmeasured states from [measured outputs], enabling output feedback
- **Input constraint:** damping coefficient limited to [c_min, c_max]

## Results
Road input: [bump / random road profile, speed, ...]

![Body acceleration](results/body_acceleration.png)

| Metric | Passive | Nonlinear H∞ (proposed) |
|---|---|---|
| RMS body acceleration | [ ] | [ ] ([−xx%]) |
| [Suspension deflection / tire load] | [ ] | [ ] |

## Repository structure
- `models/` Simulink model
- `scripts/` parameters and simulation scripts
- `results/` figures

## How to run
Tested on MATLAB/Simulink [R20xx]. Run `scripts/run_simulation.m`.

## Reference
E. Shimizu, [title of the 1999 thesis], 1999.
