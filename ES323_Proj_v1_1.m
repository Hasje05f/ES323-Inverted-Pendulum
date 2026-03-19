%% Cart-Pendulum System - State Space Model & LQR Control
% Based on Lagrangian mechanics derivation
% System: Cart (M) with pendulum (m, length L) on rail of length d
%
% States:   x = [x; x_dot; theta; theta_dot]
% Input:    u = F (force on cart)
% Output:   y = [x; theta]

clear; clc; close all;

% Insert personal directory Simulators-master, from project BelADL
dir = 'C:\Users\bouba\OneDrive\Bureaublad\ROYAL MILITARY ACADEMY\POL\3BA 2025-2026\ES322 en ES323\ES323 Project\simulators-master';
cd(dir);



%% Mathematical Model

% Lagrangian mechanics, Free Body Diagram on paper
syms t x(t) theta(t) 
syms l M m I g F b

x_p = x.*sin(theta);
y_p = l.*cos(theta);

x_pdot = diff(x_p,t,1);
y_pdot = l.*cos(theta);
v_p = sqrt(x_pdot.^2+y_pdot.^2);

% Kinetic Energy
T_cart = 0.5*M.*diff(x,t,1).^2;
T_pend_trans = 0.5.*m.*v_p.^2;
T_pend_rot = 0.5.*I.*diff(theta,t).^2;

T = T_cart + T_pend_trans + T_pend_rot;

% Potential Energy
V = m.*g.*y_p;

% Lagrangian
L = simplify(T - V);

% Euler-Lagrange  d/dt(∂L/∂q̇) - ∂L/∂q = Q_nc
    % Solving for x
    dL_dxdot  = diff(L, diff(x,t));
    ddt_xdot  = diff(dL_dxdot, t);
    dL_dx     = diff(L, x);
    
    eqn_x = F - b.*diff(x,t) == simplify(ddt_xdot - dL_dx);          
    
    % solving for theta
    dL_dthdot = diff(L, diff(theta,t));
    ddt_thdot = diff(dL_dthdot, t);
    dL_dth    = diff(L, theta);
    
    eqn_th = 0 == simplify(ddt_thdot - dL_dth);       

% Deliverables

disp('=== Equations of motion (from Euler-Lagrange, nonlineair) ===\n');
    disp('Equation for x (cart):');
    disp(eqn_x)
    disp('Equation for theta (pendulum):');
    disp(eqn_th)


%% Parameter Identification
%Preliminary checks

foundFiles = cellfun(@(f) any([exist(f,'file'), exist(fullfile(fileparts(mfilename('fullpath')) , f),'file')]==2), {'Inverted Pendulum/Parameters.m','Inverted Pendulum/PENDULUM.py'});
if any(~foundFiles)
    error('Missing required file(s): %s', strjoin(files(~ok),', '));
end

%% EXPERIMENT 1 — Pendulum Natural Frequency (cart locked)

% Build and write the JSON just as params.m
params = struct();
params.add_noise = true;
params.controller_type = 'classical';
params.trajectory = zeros(400, 1);   % experiment_1 case: cart stationary

output_file = fullfile(dir, 'Inverted Pendulum', 'controller_params.json');
fid = fopen(output_file, 'w');
fwrite(fid, jsonencode(params));
fclose(fid);

%Run Python

invDir = fullfile(dir, 'Inverted Pendulum');
batchFile = fullfile(invDir, 'run_pendulum.bat');

fid = fopen(batchFile, 'w');
if fid == -1
    error('Could not create batch file: %s', batchFile);
end
fprintf(fid, 'cd /d "%s"\n', invDir);            % change drive and directory
fprintf(fid, 'python "./PENDULUM"\n');           % run the Python script
fclose(fid);

% Execute the batch file to run the Python simulation
[status, cmdout] = system(['"', batchFile, '"']);

if status ~= 0
    warning('Batch file execution failed (status=%d). Output:\n%s', status, cmdout);
else
    fprintf('Batch file executed successfully.\n');
end









%% PARAMETERS
clear
L = 0.3;          % Pendulum total length [m]
l = L/2;          % Distance from pivot to pendulum CoM [m]
M = 0.5;          % Cart mass [kg]
m = 0.2;          % Pendulum mass [kg]
b = 0.1;          % Cart friction coefficient [kg/s]
g = 9.81;         % Gravitational acceleration [m/s^2]
c = 0.1;          % Cart length [m]
d = 2.0;          % Rail length [m] (x in [-1, 1])

% Moment of inertia of pendulum about its center of mass
I_G = m * L^2 / 12;   % = 0.0015 kg·m²

Ts = 0.05;        % Sampling frequency [s]

%% STATE-SPACE MATRICES (around theta = 0)
%Soln by Lagrange mechanics (see

M_eq = (M + m) * (m*l^2 + I_G);
Delta = M_eq - m^2 * l^2;
disp(Delta)


A = [ 0,                    1,                       0,  0 ;
      0,  -b*(m*l^2+I_G)/Delta,    g*m^2*l^2/Delta,          0 ;
      0,                    0,                       0,  1 ;
      0,      -b*m*l/Delta,         g*m*l*(M+m)/Delta,        0 ];

B = [ 0;
      (m*l^2 + I_G)/Delta;
      0;
      m*l/Delta ];

C = [ 1, 0, 0, 0 ;
      0, 0, 1, 0 ];

D = zeros(2, 1);

%% CONTROLLABILITY & OBSERVABILITY CHECK

Co = ctrb(A, B);
Ob = obsv(A, C);

    fprintf('=== System Analysis ===\n');
    fprintf('Rank of Controllability Matrix: %d (should be 4)\n', rank(Co));
    fprintf('Rank of Observability Matrix:   %d (should be 4)\n\n', rank(Ob));

%% State-Space Modeling

sys = ss(A,B,C,D,'InputName',{'F'},'OutputName',{'x','\theta'});
H = tf(sys); % Both for x and \theta respectivly
[p, z] = pzmap(sys(2,1));

    fprintf('=== State Space Continious ===\n');
    fprintf('H(2,1) transfer function:\n');
    tf(H(2,1))
    fprintf('poles: %s\n', mat2str(p.'));
    fprintf('zeros: %s\n', mat2str(z.'));

sys_d = c2d(sys, Ts);
H_d = tf(sys_d);
[p_d, z_d] = pzmap(sys_d(2,1));

    fprintf('=== State Space Discrete ===\n');
    fprintf('H_d(2,1) transfer function:\n');
    tf(H_d(2,1))
    fprintf('poles: %s\n', mat2str(p_d.'));
    fprintf('zeros: %s\n', mat2str(z_d.'));

%%
figure('Name','Continuous Step Response (all outputs)','NumberTitle','off');
    step(sys,10);

figure('Name','Continuous Pole-Zero Map (theta output)','NumberTitle','off');
    pzmap(H(2,1));

figure('Name','Discrete Step Response (all outputs)','NumberTitle','off');
    step(sys_d,10);

figure('Name','Discrete Pole-Zero Map (theta output)','NumberTitle','off');
    pzmap(H_d(2,1));

%% Comparison Simulation 

states_data = readmatrix('pendulum_states.csv');
inputs_data = readmatrix('pendulum_inputs.csv');

T = Ts;                          
N = size(states_data,1);
t_sim = (0:N-1).' * T;
x_sim = states_data(:,1);        
theta_sim = states_data(:,3);    


if ~isempty(inputs_data)
    u_sim = inputs_data(:);
    if length(u_sim) ~= N
        u_sim = interp1(linspace(0,1,length(u_sim)), u_sim, linspace(0,1,N)).';
    end
else
    u_sim = ones(N,1);          
end

x0 = [0; 0; 0.01; 0];
[y_theory, t_theory] = lsim(sys_d, u_sim, t_sim, x0);

x_theory = y_theory(:,1);
theta_theory = y_theory(:,2);

rail_max = 0.9;
rail_min = -0.95;
phi_max = pi/2.1;

theta_theory_clamped = min(max(theta_theory, -phi_max), phi_max);
x_theory_clamped = min(max(x_theory, rail_min), rail_max);

%%

figure('Name','Comparison: Python Sim vs MATLAB Linear Model','NumberTitle','off');

subplot(2,1,1);
    plot(t_sim, theta_sim, '--', 'LineWidth', 1.5); hold on;
    plot(t_theory, theta_theory_clamped, 'LineWidth', 1);
    title('Vergelijking Hoek \theta (Slinger)');
    xlabel('Tijd (s)'); ylabel('Hoek (rad)');
    legend('Python (Simulator)','MATLAB (Gelineariseerd Model)','Location','best');
    grid on;

subplot(2,1,2);
    plot(t_sim, x_sim, '--', 'LineWidth', 1.5); hold on;
    plot(t_theory, x_theory_clamped, 'LineWidth', 1);
    title('Vergelijking Positie x (Kar)');
    xlabel('Tijd (s)'); ylabel('Positie (m)');
    legend('Python (Simulator)','MATLAB (Gelineariseerd Model)','Location','best');
    grid on;
%% 
 

    

