% run_sim.m
clear; clc;

% ensure workspace params exist
if exist('setup_file.m','file')
    run('setup_file.m');
else
    warning('setup_file.m not found. Ensure g,mass,Cd,A,rho0,H,x0 exist in workspace.');
end

% build model if missing
model = 'Satellite_EDL_Controller';
if ~bdIsLoaded(model) && ~exist([model '.slx'],'file')
    if exist('build_Satellite_EDL_Controller_ready.m','file')
        run('build_Satellite_EDL_Controller_ready.m');
    else
        error('Model and build script not found. Place build_Satellite_EDL_Controller_ready.m in current folder and run again.');
    end
end

% simulation inputs (root inports order: AltitudeRef, Disturbance)
t0 = 0; tf = 200; dt = 0.5;
T = (t0:dt:tf)';                    % time column
AltRef_val = 0;                     % desired landing altitude (m)
AltRef = AltRef_val*ones(size(T));  % constant reference
Disturb = zeros(size(T));           % disturbance (N)
extIn = [T AltRef Disturb];         % [time, in1, in2]

% run simulation
simOut = sim(model, 'ExternalInput', 'extIn', 'SrcWorkspace', 'current', 'SaveOutput','on', 'SaveTime','on');

% retrieve To Workspace logs if present
if exist('sim_alt','var') && exist('sim_vel','var')
    t_alt = sim_alt.time;
    z = sim_alt.signals.values;      % altitude
    t_vel = sim_vel.time;
    v = sim_vel.signals.values;      % velocity
else
    % try to extract from simOut
    try
        logsout = simOut.get('simout');
    catch
        logsout = [];
    end
    if isempty(logsout)
        warning('No workspace logs found (sim_alt/sim_vel). Check model To Workspace blocks.');
        return
    else
        t_alt = simOut.tout;
        z = simOut.get('yout'); % fallback
        v = [];
    end
end

figure;
subplot(2,1,1);
plot(t_alt, z);
ylabel('Altitude (m)');
grid on;
title('Altitude vs Time');

subplot(2,1,2);
plot(t_vel, v);
ylabel('Velocity (m/s)');
xlabel('Time (s)');
grid on;

disp('Simulation complete.');
