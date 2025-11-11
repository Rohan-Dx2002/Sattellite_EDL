%% build_Satellite_EDL_Controller_ready.m
clear; clc;

model = 'Satellite_EDL_Controller';
if bdIsLoaded(model), close_system(model, 0); end
new_system(model); open_system(model);
set_param(model, 'Solver', 'ode45', 'StopTime', '200', 'SaveFormat','StructureWithTime');

left = 30; top = 30; w = 140; h = 50; dx = 240; dy = 120;

add_block('simulink/Sources/In1', [model '/AltitudeRef'], 'Position', [left top left+w top+h]);
add_block('simulink/Sources/In1', [model '/Disturbance'], 'Position', [left top+dy left+w top+dy+h]);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/Controllers'], 'Position', [left+dx top left+dx+w top+h+40]);
open_system([model '/Controllers']);

add_block('simulink/Sources/In1', [model '/Controllers/AltRef_in'], 'Position', [20 20 60 40]);
add_block('simulink/Sources/In1', [model '/Controllers/Alt_in'], 'Position', [20 80 60 100]);
add_block('simulink/Sources/In1', [model '/Controllers/Vel_in'], 'Position', [20 140 60 160]);
add_block('simulink/Continuous/PID Controller', [model '/Controllers/PID_alt'], 'Position', [140 20 260 70], 'P', '600', 'I', '5', 'D', '50', 'N', '50');

mpc_code = [
"function u = mpc_controller(alt_ref, alt, vel, mass, g)\n"...
"dt=0.5;N=12;A=[1 dt;0 1];B=[0;dt/mass];Q=diag([100,1]);R=0.001;\n"...
"nx=2;nu=1;Qb=kron(eye(N),Q);Rb=kron(eye(N),R);\n"...
"Phi=zeros(nx*N,nx);Gamma=zeros(nx*N,nu*N);\n"...
"for k=1:N;Phi((k-1)*nx+1:k*nx,:)=A^k;for j=1:k;Gamma((k-1)*nx+1:k*nx,(j-1)*nu+1:j*nu)=A^(k-j)*B;end;end\n"...
"z0=alt_ref-alt;x0=[z0;-vel];ref=zeros(nx*N,1);\n"...
"H=Gamma'*Qb*Gamma+Rb;f=Gamma'*Qb*(Phi*x0-ref);\n"...
"umax=2*mass*g;umin=0;lb=repmat(umin,N,1);ub=repmat(umax,N,1);\n"...
"try;opts=optimoptions('quadprog','Display','off');U=quadprog((H+H')/2,f,[],[],[],[],lb,ub,[],opts);u=U(1);\n"...
"catch;P=zeros(nx,nx,N+1);P(:,:,N+1)=Q;for k=N:-1:1;S=R+B'*P(:,:,k+1)*B;K=(S)\\(B'*P(:,:,k+1)*A);P(:,:,k)=A'*P(:,:,k+1)*(A-B*K)+Q;end;K0=(R+B'*P(:,:,1)*B)\\(B'*P(:,:,1)*A);u_fb=-K0*x0;u=max(umin,min(umax,u_fb));end\n"...
"end"
];
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/Controllers/MPC_Block'], 'Position', [140 100 420 220]);
set_param([model '/Controllers/MPC_Block'], 'FunctionName', 'mpc_controller');
set_param([model '/Controllers/MPC_Block'], 'Script', mpc_code);

add_block('simulink/Math Operations/Sum', [model '/Controllers/SumCtrl'], 'Position', [300 40 350 80], 'Inputs', '+-');
add_block('simulink/Signal Routing/Outport', [model '/Controllers/Thrust_out'], 'Position', [500 60 540 80]);

add_line([model '/Controllers'], 'AltRef_in/1', 'PID_alt/1');
add_line([model '/Controllers'], 'AltRef_in/1', 'MPC_Block/1');
add_line([model '/Controllers'], 'Alt_in/1', 'MPC_Block/2');
add_line([model '/Controllers'], 'Vel_in/1', 'MPC_Block/3');
add_line([model '/Controllers'], 'PID_alt/1', 'SumCtrl/1');
add_line([model '/Controllers'], 'MPC_Block/1', 'SumCtrl/2');
add_line([model '/Controllers'], 'SumCtrl/1', 'Thrust_out/1');
close_system([model '/Controllers']);

add_block('simulink/Ports & Subsystems/Subsystem', [model '/EDL_Dynamics'], 'Position', [left+2*dx top left+2*dx+w top+h+40]);
open_system([model '/EDL_Dynamics']);
add_block('simulink/Sources/In1', [model '/EDL_Dynamics/Thrust_in'], 'Position', [30 30 90 50]);
add_block('simulink/Sources/In1', [model '/EDL_Dynamics/Disturb_in'], 'Position', [30 90 90 110]);
add_block('simulink/Continuous/Integrator', [model '/EDL_Dynamics/Integr_alt'], 'Position', [260 20 320 60], 'InitialCondition', 'x0(1)');
add_block('simulink/Continuous/Integrator', [model '/EDL_Dynamics/Integr_vel'], 'Position', [260 90 320 130], 'InitialCondition', 'x0(2)');
add_block('simulink/Continuous/Integrator', [model '/EDL_Dynamics/Integr_pitch'], 'Position', [260 160 320 200], 'InitialCondition', 'x0(3)');

edl_code = [
"function [alt_dot, vel_dot, pitch_dot, alt, vel, pitch] = edl_plant(u, w, alt_s, vel_s, pitch_s)\n"...
"alt=alt_s;vel=vel_s;pitch=pitch_s;\n"...
"rho=rho0*exp(-alt/H);\n"...
"Drag=0.5*rho.*vel.^2.*Cd.*A.*sign(vel);\n"...
"vel_dot=(-mass*g+u-Drag+w)/mass;\n"...
"alt_dot=-vel;\n"...
"pitch_dot=-0.15*pitch+0.0005*u/mass;\n"...
"end"
];
add_block('simulink/User-Defined Functions/MATLAB Function', [model '/EDL_Dynamics/EDL_Plant'], 'Position', [120 40 220 220]);
set_param([model '/EDL_Dynamics/EDL_Plant'], 'Script', edl_code);
set_param([model '/EDL_Dynamics/EDL_Plant'], 'FunctionName', 'edl_plant');

add_line([model '/EDL_Dynamics'], 'Thrust_in/1', 'EDL_Plant/1');
add_line([model '/EDL_Dynamics'], 'Disturb_in/1', 'EDL_Plant/2');
add_line([model '/EDL_Dynamics'], 'Integr_alt/1', 'EDL_Plant/3');
add_line([model '/EDL_Dynamics'], 'Integr_vel/1', 'EDL_Plant/4');
add_line([model '/EDL_Dynamics'], 'Integr_pitch/1', 'EDL_Plant/5');
add_line([model '/EDL_Dynamics'], 'EDL_Plant/1', 'Integr_alt/1');
add_line([model '/EDL_Dynamics'], 'EDL_Plant/2', 'Integr_vel/1');
add_line([model '/EDL_Dynamics'], 'EDL_Plant/3', 'Integr_pitch/1');

add_block('simulink/Ports & Subsystems/Out1', [model '/EDL_Dynamics/alt_out'], 'Position', [360 20 400 40]);
add_block('simulink/Ports & Subsystems/Out1', [model '/EDL_Dynamics/vel_out'], 'Position', [360 80 400 100]);
add_block('simulink/Ports & Subsystems/Out1', [model '/EDL_Dynamics/pitch_out'], 'Position', [360 140 400 160]);
add_line([model '/EDL_Dynamics'], 'Integr_alt/1', 'alt_out/1');
add_line([model '/EDL_Dynamics'], 'Integr_vel/1', 'vel_out/1');
add_line([model '/EDL_Dynamics'], 'Integr_pitch/1', 'pitch_out/1');
close_system([model '/EDL_Dynamics']);

add_block('simulink/Sinks/Scope', [model '/Scope_alt'], 'Position', [left+3*dx top left+3*dx+160 top+80]);
add_block('simulink/Sinks/Scope', [model '/Scope_vel'], 'Position', [left+3*dx top+100 left+3*dx+160 top+180]);

add_line(model, 'AltitudeRef/1', 'Controllers/1');
add_line(model, 'Controllers/1', 'EDL_Dynamics/1');
add_line(model, 'Disturbance/1', 'EDL_Dynamics/2');
add_line(model, 'EDL_Dynamics/1', 'Controllers/2');
add_line(model, 'EDL_Dynamics/2', 'Controllers/3');
add_line(model, 'EDL_Dynamics/1', 'Scope_alt/1');
add_line(model, 'EDL_Dynamics/2', 'Scope_vel/1');

add_block('simulink/Sinks/To Workspace', [model '/Log_alt'], 'Position', [left+3*dx+200 top left+3*dx+340 top+40], 'VariableName', 'sim_alt', 'SaveFormat','StructureWithTime');
add_block('simulink/Sinks/To Workspace', [model '/Log_vel'], 'Position', [left+3*dx+200 top+100 left+3*dx+340 top+140], 'VariableName', 'sim_vel', 'SaveFormat','StructureWithTime');
add_line(model, 'EDL_Dynamics/1', 'Log_alt/1');
add_line(model, 'EDL_Dynamics/2', 'Log_vel/1');

save_system(model, [model '.slx']);
open_system(model);
disp(['Model saved: ' model '.slx']);
