 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A3S EXAM SCRIPT AY 2025/2026
% Author:  Davide Invernizzi (davide.invernizzi@polimi.it)
% v09/10/2025      
% This file contains data for the exam of the A3S course ay 2025/2026.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;
addpath("Task2/")
%% parameters

par.g = 9.81;
par.e3 = [0 0 1]';

% inital conditions
p_i_0 = [0 0 1]';
v_i_0 = [0 0 0]';

omega_p = 1;
xi_p = 1;


k_p = omega_p^2; 
k_v = 2*xi_p*omega_p;

degToRad = pi/ 180; 
radTodeg = 1/degToRad; 

%% initial condition


q_0 = eul2quat([0, 0, 0 ]*degToRad,'XYZ')'; %  attitude - quaternion
omegab_0 = [0 0 0]'; %  rad/s  angular velocity (body components)
p_0 = [0 0 0]'; % m position
vb_0 = [0 0 0]'; % m/s linear velocit

%% UAV inertial and dynamic parameters
% Nominal values for the parameters

UAV.Nr = 6; %number of rotors

UAV.m = 1.2; %[kg] UAV mass

UAV.J = diag([0.01 0.01 0.018]); % [kg m^2] inertia matrix
UAV.Jinv = inv(UAV.J); 

UAV.r_bg = [0 0 0]; %[m] center of mass location in the body frame
UAV.S = UAV.m*crossmat(UAV.r_bg); % [kg m] static moment


UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
         UAV.S UAV.J];

UAV.Minv = inv(UAV.M); % inverse of mass matrix

% Linear Aerodynamics
UAV.D_tauomega = diag([0.48 0.48 2.37]); %  angular velocity damping 
UAV.D_fv =  diag([0.055 0.055 0.022]); % linear velocity damping

% UAV.D = [UAV.D_tauomega zeros(3,3); zeros(3,3) UAV.D_fv];
UAV.D = [UAV.D_fv zeros(3,3); zeros(3,3) UAV.D_tauomega];

%% Propellers 

UAV.b = 0.215; %[m] arm length (ell in the slides)

UAV.Omega_max = 10300 * 2 * pi /60; %[rad/s] max spinning rate
UAV.Omega_min = 1260  * 2 * pi /60; %[rad/s] min spinning rate


UAV.k_m = 1/0.05; % [s-1] Inverse of the time constant of the propeller motors
k_f = 3.65e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 0.09;      % [m] Torque-to-thrust ratio

lambda = diag(ones(6,1));

gamma = 360/UAV.Nr;
i = 1:6;
row1 = UAV.b * sind(gamma*(i-1)); 
row2 = -UAV.b*cosd(gamma*(i-1));


direction = [1 -1 1 -1 1 -1];

F = [zeros(2,6);ones(1,6); row1; row2; direction * sigma];
%% Control allocation parameter 

min.F1 = [zeros(2,4); eye(4)];


min.F2 = F(3:end,:); 

min.F2_pi = min.F2'/(min.F2*min.F2');

min.P2 = eye(6) - min.F2_pi * min.F2; 

min.Tmin = UAV.Omega_min^2 * k_f; 
min.Tmax = UAV.Omega_max^2 * k_f; 



%% SIMULATION 

% trim condition for position 
 cmd = sqrt(UAV.m * par.g / 6 / k_f);
 cmd = ones(6,1) * cmd;

% theta_cmd =  asind(2 *(  UAV.D_fv(1,1))/UAV.m/par.g );
% vb_0 = [2 0 0]'; % m/s linear velocit
% 
% 
% q_0 = eul2quat([0, theta_cmd, 0 ]*degToRad,'ZYX')'; %  attitude - quaternion
% 
% 
% cmd_4 = sqrt(UAV.m * par.g * cosd(theta_cmd)/6 /k_f); 
% 
% cmd = ones(6,1)* cmd_4;

stop_time = 20;


% simout = sim('dyn.slx','StopTime','stop_time');
% %% PLOT RESULTS
% figure
% plot(simout.euler * radTodeg,'LineWidth',3);
% grid minor
% xlabel('[s]')
% ylabel('[deg]')
% title('Attitude')
% legend('\phi', '\theta', '\psi')
% 
% figure
% plot(simout.omegab,'LineWidth',3);
% grid minor
% xlabel('[s]')
% ylabel('[rad/s]')
% title('Angular velocity')
% legend('p', 'q', 'r')
% 
% 
% figure
% plot(simout.p,'LineWidth',3);
% grid minor
% xlabel('[s]')
% ylabel('[m]')
% title('Position NED frame')
% legend('x', 'y', 'z')
% 
% figure
% plot(simout.vb,'LineWidth',3);
% grid minor
% xlabel('[s]')
% ylabel('[m/s]')
% title('Linear body velocity')
% legend('v_x', 'v_y', 'v_z')



function x_cross = crossmat(x)
x_cross=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end
