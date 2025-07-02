% Satellite EDL Simulation Setup
clear; clc;

%% Physical Parameters
g = 9.81;           % Gravity (m/s^2)
mass = 500;         % Satellite mass (kg)
Cd = 1.2;           % Drag coefficient
A = 1.5;            % Cross-sectional area (m^2)
rho0 = 1.225;       % Air density at sea level (kg/m^3)
H = 8500;           % Scale height (m)

%% Initial States
x0 = [10000; 0; 0];  % [Altitude (m); Velocity (m/s); Pitch angle (rad)]

%% Create Simulink model
simulink_model = ['Satellite_EDL_Controller.slx'];
open_system(simulink_model)
