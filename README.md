# Non-linear-Hinfinity-with-observer-version1


# Nonlinear H∞ Output-Feedback Control for Semi-Active Suspension of an In-Wheel-Motor Vehicle

Bachelor's thesis project, Chiba institute of technology (2022). MATLAB/Simulink.

## Overview
In-wheel-motor (IWM) vehicles place the motor inside the wheel, which increases unsprung mass and degrades ride comfort and road holding. This project designs a nonlinear H∞ output-feedback controller for a semi-active suspension to suppress vertical vibration in a quarter-car model.

## Why a bilinear system
A semi-active damper can only change its damping coefficient. The damper force is the product of the control input (the damping coefficient) and a state (the relative velocity), so the system is bilinear, not linear. I applied the nonlinear H∞ control theory for bilinear systems proposed by Etsuro Shimizu from Tokyo University of Marine Science and Technology (1999).

## Method
- **Model:** 2-DOF quarter-car with in-wheel motor (sprung mass 300 kg, unsprung mass 50 kg incl. IWM,
  tire stiffness 160 kN/m, suspension stiffness 16 kN/m). Sprung/unsprung resonances ≈ 1.16 / 9.4 Hz.
- **Generalized plant:** frequency weights on body acceleration, suspension stroke, and body velocity
  (low-passed at 1 Hz), plus input weights → 16 states.
- **Controller:** nonlinear H∞ output feedback for bilinear systems (Shimizu, Ch. 5, Type 1),
  observer gain from an LMI (Lemma 5.3, γ1 = 1.5), controller from a Riccati equation (Lemma 5.4,
  γ2 = 1.05·γ2,min). The extra nonlinear weight m1(x̂) is set to 0 in this version.
- **Observer:** estimates all states from suspension stroke, body acceleration, and stroke velocity.
- **Input constraint:** damping coefficient limited to [300, 8000] Ns/m (saturation on the H∞ command).


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
