# Satellite EDL Controller

## Overview
This project builds and simulates a **Satellite Entry, Descent, and Landing (EDL)** control system using **PID** and **MPC** controllers in **Simulink**.  
It demonstrates autonomous trajectory tracking, stability control, and precision landing under realistic atmospheric and dynamic conditions.

The scripts automatically create a Simulink model (`Satellite_EDL_Controller.slx`), tune a PID controller, implement an MPC controller using `quadprog` (with LQR fallback), and generate altitude/velocity plots after simulation.

---

## Project Structure
| File | Description |
|------|--------------|
| `setup_file.m` | Defines physical parameters (mass, drag, gravity, etc.) and initial states. |
| `build_Satellite_EDL_Controller_ready.m` | Builds and saves the Simulink model (`Satellite_EDL_Controller.slx`). |
| `run_sim.m` | Runs the simulation, provides reference inputs, and plots altitude & velocity. |
| `Satellite_EDL_Controller.slx` | Generated Simulink model (automatically created). |

---

## Requirements
- **MATLAB + Simulink** (R2019b or newer)
- **Optimization Toolbox** (optional, for `quadprog` in MPC)
- Windows/macOS/Linux (64-bit)

If Optimization Toolbox is not installed, the MPC block automatically switches to a finite-horizon LQR controller.

---

## Setup Instructions

1. Open **MATLAB**.
2. Set your current working directory to the folder containing all project files.
3. Open and verify the contents of `setup_file.m` — it defines all required parameters for the simulation:

   ```matlab
   g = 9.81;          % Gravity (m/s^2)
   mass = 500;        % Satellite mass (kg)
   Cd = 1.2;          % Drag coefficient
   A = 1.5;           % Cross-sectional area (m^2)
   rho0 = 1.225;      % Air density at sea level (kg/m^3)
   H = 8500;          % Scale height (m)
   x0 = [10000; 0; 0]; % Initial state [Altitude; Velocity; Pitch angle]
4. Go to the command line and type in order.
 ```matlab
   run('setup_file.m')
   run('build_Satellite_EDL_Controller_ready.m')
   run('run_sim.m')

