%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% A3S EXAM SCRIPT AY 2025/2026
% Davide Invernizzi (davide.invernizzi@polimi.it)
% Authors: Fontana Anna, Gerardini Giulio, Orlandini Aurora      
% This file contains data for the exam of the A3S course ay 2025/2026.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;

%% parameters

par.g = 9.81;
par.e3 = [0 0 1]';

degToRad = pi/ 180; 
radTodeg = 1/degToRad; 
 

% Sensors delay 

delay.position = 0.03; 
delay.speed = 0.03; 
delay.attitude = 0.01;

%% initial conditions

q_0 = eul2quat([0, 0, 0 ]*degToRad,'ZYX')'; %  attitude - quaternion
omegab_0 = [0 0 0]'; %  rad/s  angular velocity (body components)
p_0 = [0 0 0]'; % m positionmax 
vb_0 = [0 0 0]'; % m/s linear velocity

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

UAV.k_m = 1/0.05; % [s-1] Inverse of the time constant of the propeller motors
k_f = 3.65e-6;                % [N/rad^2/s^2] Thrust characteristic coeff
sigma = 0.09;      % [m] Torque-to-thrust ratio

UAV.Omega_max = 10300 * 2 * pi /60; %[rad/s] max spinning rate
UAV.Omega_min = 1260  * 2 * pi /60; %[rad/s] min spinning rate
UAV.Tmax = k_f * UAV.Omega_max ^2; 
UAV.Tmin = k_f * UAV.Omega_min ^2; 

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

min.F2_pi = min.F2'*inv(min.F2*min.F2');

min.P2 = eye(6) - min.F2_pi * min.F2; 

min.Tmin = ones(6,1) * UAV.Omega_min^2 * k_f; 
min.Tmax = ones(6,1) * UAV.Omega_max^2 * k_f; 

%% Package
pack.m = 0.5; % [kg] 
pack.h = 0.03; %[m]
pack.l = 0.2; 
pack.w = 0.1; %[m] 

pack.rc = [0;0;-0.1];

pack.S = pack.m*crossmat(pack.rc); 

%% Adaptive part

adapt.Ap = [zeros(3,3), eye(3); zeros(3,3), zeros(3,3)]; 
adapt.Bp = [zeros(3,3); eye(3)]/UAV.m; 
adapt.Bpt  = adapt.Bp';
adapt.wind=[10;10;0]*0; % wind gust disturbance
adapt.phit= eye(3);
adapt.L = eye(6)*15;
adapt.Gamma_a = eye(6) * 0.1;


%% Delivery trajectory
% trajectory loading
load('traiettoria_finale.mat');
disp('Loading trajectory')

%% TRIM TASK 1

% position trim
p_0=[0,0,1]';
cmd = sqrt(UAV.m * par.g / 6 / k_f);
cmd = ones(6,1) * cmd;
stop_time = 10;

simout = sim('task1.slx','StopTime','stop_time');

plot_graph(simout)

% velocity trim
theta_cmd =  asind(2 *(UAV.D_fv(1,1))/UAV.m/par.g );
vb_0 = [2 0 0]'; % m/s linear velocit

q_0 = eul2quat([0, theta_cmd, 0 ]*degToRad,'ZYX')'; %  attitude - quaternion

cmd_4 = sqrt(UAV.m * par.g * cosd(theta_cmd)/6 /k_f);

cmd = ones(6,1)* cmd_4;

stop_time = 10;

simout = sim('task1.slx','StopTime','stop_time');
plot_graph(simout)

%% TASK 2

% attitude
par.eul_d = [10; 10 ; 10]'*degToRad;
stop_time = 10;

s = tf('s');
% x and y axes
o_ref = 15; 
csi_ref = 0.9;  
ref = o_ref^2/(s^2 + 2*s*csi_ref*o_ref + o_ref^2);

ctrl.Komega = 2 * o_ref * csi_ref * UAV.J;

sq_KR = diag(o_ref ^ 2 * UAV.J); 

par.k_R3 = (sq_KR(2)  - sq_KR(3) + sq_KR(1))/2 ;
par.k_R1 = sq_KR(2)  - par.k_R3;
par.k_R2 = sq_KR(1)  - par.k_R3;

simout = sim('task2_att.slx','StopTime','stop_time');
figure
step(ref) 
hold on
plot(simout.euler * radTodeg/10,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[deg]')
title('Attitude')
legend('\psi', '\theta', '\phi')

%% position 

% (made with simulink control system tuner) 
% requirements: 
%   - Loop tuning: omega_c=0.8 rad/s [HARD]
%   - Step tracking: I ordine, Ts=1 s
%   - Step rejection: max amplitude =1
%                     max Ts=1
%                     max csi=1

ctrl.K_p=2.4201430996143;
ctrl.K_v=2.22888123838358;
ctrl.K_i=1.23118028644017;

stop_time = 30;

simout = sim('task2_pos.slx','StopTime','stop_time');


figure
plot(simout.p_i,'LineWidth',3);
grid minor
xlabel('[s]')
ylabel('[m]')
title('Position Inertial frame')
legend('x', 'y', 'z')

%% full system

flag_PBMRAC = 0;
traj_case = 2; % 1 for circle 2 for step
p_0 = zeros(3,1); 
vb_0 = zeros(3,1); 

simout = sim('task3_MRAC3_finale.slx','StopTime','stop_time');
plot_ep(simout)
plot_graph(simout)
[rmse_old, rmse2_old] = RMSE_compute(simout);


%% retune attitude controller

% z axis
o_ref_psi = 15; 
csi_ref_psi= 0.9;  
s = tf('s');
ref_psi = o_ref_psi^2/(s^2 + 2*s*csi_ref_psi*o_ref_psi+ o_ref_psi^2);

% x and y axes
o_ref = 15; 
csi_ref = 0.1;  
ref = o_ref^2/(s^2 + 2*s*csi_ref*o_ref + o_ref^2);

ctrl.Komega = 2 * o_ref * csi_ref * UAV.J;
ctrl.Komega(3,3) = 2 * o_ref_psi * csi_ref_psi * UAV.J(3,3); 

sq_KR = diag(o_ref ^ 2 * UAV.J); 
sq_KR(3) = o_ref_psi ^2 * UAV.J(3,3); 

par.k_R3 = (sq_KR(2)  - sq_KR(3) + sq_KR(1))/2 ;
par.k_R1 = sq_KR(2)  - par.k_R3;
par.k_R2 = sq_KR(1)  - par.k_R3;

%% full system retuned

flag_PBMRAC = 0;
traj_case = 1; % 1 for circle 2 for step
%lambda = diag(ones(6,1));

p_0 = zeros(3,1); 
vb_0 = zeros(3,1); 

simout = sim('task3_MRAC3_finale.slx','StopTime','stop_time');
plot_ep(simout)
plot_graph(simout)
[rmse, rmse2] = RMSE_compute(simout);

%% task 3

UAV = include_pack(UAV,pack,1); % 1 to include package 0 to exclude it
flag_PBMRAC = 1;
traj_case = 2; % 1 for circle 2 for step 3 for sinusoid
adapt.wind=[10;10;0]*1; % wind gust disturbance

stop_time = 30;
simout = sim('task3_MRAC3_finale.slx','StopTime','stop_time');
plot_ep(simout)
plot_graph(simout)
[rmse, rmse2] = RMSE_compute(simout);


%% DELIVERY SCENARIO TASK 4

close all 

p_0 = [17, 22.5, 0]'; % uncomment for the delivery trajectory

UAV = include_pack(UAV,pack,1); % 1 to include package 0 to exclude it
flag_PBMRAC = 0;
traj_case = 4; % 4 for delivery (check p_0) 
adapt.wind=[10;10;0]*0; % wind gust disturbance
lambda = diag(ones(6,1))*0.6;

stop_time = 80; % stop_time = 80 for the delivery trajectory

simout = sim('task3_MRAC3_finale.slx','StopTime','stop_time');
plot_ep(simout)
plot_graph(simout)
[rmse, rmse2] = RMSE_compute(simout);


% plot trajectory in 3D
planned = squeeze(simout.planned.Data)';
x = squeeze(simout.p.data(1,1,:));
y = squeeze(simout.p.data(2,1,:)); 
z = squeeze(simout.p.data(3,1,:));

figure
plot3(planned(:,1), planned(:,2) , planned(:,3)); 
hold on
plot3(x,y,z);

res = 1; 

% Pansushi 
length_box = 5;
width_box = 5;
height_box = 10;
pos_x1 = 17;
pos_y1 = 17;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruito Pansushi \n')

% Galleria del vento
length_box = 20;
width_box = 7;
height_box = 15;
pos_x1 = 15;
pos_y1 = 4;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruita la galleria del vento \n')

% B19
length_box = 10;
width_box = 10;
height_box = 15;
pos_x1 = 4;
pos_y1 = 15;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruit0 B19 \n')

% box B20
length_box = 10;
width_box = 7;
height_box = 15;
pos_x1 = -10;
pos_y1 = 15;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruito B20 \n')

% Collinetta
length_box = 10;
width_box = 15;
height_box = 10;
pos_x1 = -5;
pos_y1 = -12.5;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruita la Collinetta \n')

% box B12
length_box = 10;
width_box = 25;
height_box = 15;
pos_x1 = -20;
pos_y1 = -12.5;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruito B12 \n')

% box B14-B16
length_box = 20;
width_box = 20;
height_box = 10;
pos_x1 = 15;
pos_y1 = -15;
plot_building(res, pos_x1, pos_y1, length_box, width_box, height_box) % rectangular obstacle
fprintf('Costruiti edifici da B14 a B16 \n')


%% functions

function UAV = include_pack(UAV,pack,flag)
    
    UAV.M = [UAV.m*eye(3) UAV.S'; % generalized mass matrix
         UAV.S UAV.J];

    UAV.M = UAV.M + flag * diag([pack.m, pack.m, pack.m, 1/12 * pack.m * (pack.h^2 + pack.l^2)+ pack.m *(pack.rc(1)^2 + pack.rc(3)^2) , 1/12 * pack.m * (pack.h^2 + pack.w^2)+ pack.m *(pack.rc(2)^2 + pack.rc(3)^2)  , 1/12 * pack.m*(pack.l^2 + pack.w ^2)]); 
    UAV.M = UAV.M + flag * [zeros(3) , pack.S'; pack.S, zeros(3)];
    
    UAV.Minv = inv(UAV.M);
end

function [] = plot_graph(simout)
    degToRad = pi/ 180; 
    radTodeg = 1/degToRad; 
    figure
    subplot(2,2,1);
    plot(simout.euler * radTodeg,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[deg]')
    title('Attitude')
    legend('\psi', '\theta', '\phi')
    
    subplot(2,2,2);
    plot(simout.omegab,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[rad/s]')
    title('Angular velocity')
    legend('p', 'q', 'r')
    
    subplot(2,2,3);
    plot(simout.p,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position ENU frame')
    legend('x', 'y', 'z')
    
    subplot(2,2,4);
    plot(simout.vi,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[m/s]')
    title('Linear body velocity')
    legend('v_x', 'v_y', 'v_z')

   
end

function [] = plot_ep(simout)
     figure
    plot(simout.e_p,'LineWidth',3);
    grid minor
    xlabel('[s]')
    ylabel('[m]')
    title('Position error - ENU frame')
    legend('x', 'y', 'z')
end

function [rmse,rmse_2] = RMSE_compute(simout) 
    time=simout.tout;
    time_2 = time > 10;
    x = squeeze(simout.p.data(1,1,:));
    y = squeeze(simout.p.data(2,1,:)); 
    z = squeeze(simout.p.data(3,1,:));
    
    % Root mean square error
    planned = squeeze(simout.planned.Data)'; 
    error_sq = sqrt((planned(:,1) - x).^2 + (planned(:,2) - y).^2 + (planned(:,3) - z).^2); 
    rmse = sqrt(mean(error_sq));
    error_sq_2 = sqrt((planned(time_2,1) - x(time_2)).^2 + (planned(time_2,2) - y(time_2)).^2 + (planned(time_2,3) - z(time_2)).^2); 
    rmse_2 = sqrt(mean(error_sq_2));
end

function x_cross = crossmat(x)
x_cross=[0 -x(3) x(2);
    x(3) 0 -x(1);
    -x(2) x(1) 0];
end

function [] = plot_building(res, pos_x, pos_y,length, width, height)  
x_inter = pos_x - length/2+res/2: res : pos_x + length/2-res/2;
y_inter = pos_y - width/2+res/2:res:pos_y+ width/2-res/2;

for x = [x_inter(1), x_inter(end)]
    for y = y_inter
        for z = 0:res:round(height, 1)
            
            plot3(x,y,z,'b*')
        end
    end
end

for x = x_inter
    for y = [y_inter(1), y_inter(end)]
        for z = 0:res:round(height, 1)
            
            plot3(x,y,z,'b*')
        end
    end
end
end