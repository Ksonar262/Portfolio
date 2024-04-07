%% 

clear ; clear all; close all;
% Read data files
data = readtable("input_data_cubic.xlsx");

%%
%extract data from the table to make into required vectors
Ts=10;
t=data.Time;
y_k = [data.LFIN_x data.LFIN_y data.LFIN_z];
c_k = [data.q1 data.q2 data.q3 data.q4 data.LSHO_x data.LSHO_y data.LSHO_z];

%initialisation of matrices and variables
xhat_km1_km1 = [-1010.67 -970.791 690.5866];
P_km1_km1 = diag([1000 1000 100]);
Q=diag([0.001 0.001 0.001 0.001 0.001 0.001 0.001]);
R=diag([1 1 1 1 1 1 1]);
u_km1 = [0 0 0 0 0 0 0; c_k];
N = length(t);
n=length(xhat_km1_km1);
m=size(y_k, 2);
v= [0.0372, 0.0271, 0.0278];

% Preallocate the storage
stdx_cor = zeros(N, n);
x_cor = zeros(N, n);
K_k = cell(N, 1);
innov = zeros(N, m);


%%
%Kalman filter loop
for k=1:N
% Step 1: Prediction
x_dot = funcf (xhat_km1_km1 , u_km1 (k ,:)); % prediction equation
xhat_k_km1 = xhat_km1_km1 + x_dot *Ts; % x(k|k -1) ( prediction ) Euler integration
yhat_k_km1 = funch( xhat_k_km1) ; % y(k|k -1) ( prediction )

% Step 2: Covariance matrix of state prediction error / Minimum prediction MSE
[ Phi_km1 , Gamma_km1 ] = funcLinF (u_km1 (k ,:)); % Phi(k -1) , Gamma (k -1)
P_k_km1 = Phi_km1 * P_km1_km1 * Phi_km1' + Gamma_km1 * Q * Gamma_km1'; % P(k|k -1) ( prediction )

% Step 3: Kalman Gain
H_k = funcLinH (); %H(k)
Ve = (H_k * P_k_km1 * H_k' + R);
K = P_k_km1 * H_k' / Ve; % K(k) ( gain )

% Step 4: Measurement Update ( Correction )
xhat_k_k = xhat_k_km1 + (y_k(k ,:) - yhat_k_km1 )*K'; % x(k|k) (correction )

% Step 5: Correction for Covariance matrix of state Estimate error / Minimum MSE
I_KH = eye (n) - K * H_k;
P_k_k = I_KH * P_k_km1 * I_KH' + K * R * K'; % P(k|k) (correction )

% Save intermediate data , for analysis later
stdx_cor (k ,:) = sqrt ( diag ( P_km1_km1 )); % \ sigma (k -1|k -1) , the standard deviation of state estimation error
x_cor (k ,:) = xhat_km1_km1 ; % \hat{x}(k -1|k -1) , estimated state
K_k{k ,1} = K; % K(k) Kalman Gain
innov (k ,:)= y_k(k ,:) - yhat_k_km1 ; %y(k) -\hat{y}(k|k -1) , innovation , with \hat{y}(k|k -1)=h(\ hat{x}(k|k -1));

% Recursive step
xhat_km1_km1 = xhat_k_k ;
P_km1_km1 = P_k_k ;
end

%%

function x_dot = funcf(x, u_c)

[ha_x, ha_y, ha_z] = deal(x(1), x(2), x(3));
[q1, q2, q3, q4, sh_x, sh_y, sh_z] = deal(u_c(1), u_c(2), u_c(3), u_c(4), u_c(5), u_c(6), u_c(7));


ha_x_dot = (253*cos(q1)*cos(q4)^2)/20 + (161*cos(q1)*cos(q4))/10 - (253*cos(q1)*sin(q4)^2)/20 + ha_x/2 + sh_x/2 + (93/500);
ha_y_dot = ha_y/2 + sh_y/2 + (161*cos(q3)*sin(q4))/10 + (253*cos(q4)*(cos(q3)*sin(q4) + cos(q4)*sin(q1)*sin(q3)))/20 + (253*sin(q4)*(cos(q3)*cos(q4) - sin(q1)*sin(q3)*sin(q4)))/20 + (161*cos(q4)*sin(q1)*sin(q3))/10 + (271/2000);
ha_z_dot = ha_z/2 + sh_z/2 + (161*sin(q3)*sin(q4))/10 + (253*cos(q4)*(sin(q3)*sin(q4) - cos(q3)*cos(q4)*sin(q1)))/20 + (253*sin(q4)*(cos(q4)*sin(q3) + cos(q3)*sin(q1)*sin(q4)))/20 - (161*cos(q3)*cos(q4)*sin(q1))/10 + (139/1000);

x_dot = [ha_x_dot ha_y_dot ha_z_dot];

end

function y = funch(x)
ha_x_vicon = x(1);
ha_y_vicon = x(2);
ha_z_vicon = x(3);
y=[ha_x_vicon ha_y_vicon ha_z_vicon];
end

function [Phi, Gamma] = funcLinF(u_c)
omega_q1 = 0.4;
omega_q2 = 0.3;
omega_q3 = 0.5;
omega_q4 = 0.5;

[q1_m, q2_m, q3_m, q4_m, sh_x_m, sh_y_m, sh_z_m] = deal(u_c(1), u_c(2), u_c(3), u_c(4), u_c(5), u_c(6), u_c(7));


A= 1/2 * eye(3);

G = [- (253*sin(omega_q1 - q1_m)*cos(omega_q4 - q4_m)^2)/20 - (161*sin(omega_q1 - q1_m)*cos(omega_q4 - q4_m))/10 + (253*sin(omega_q1 - q1_m)*sin(omega_q4 - q4_m)^2)/20, 0, 0, - (161*cos(omega_q1 - q1_m)*sin(omega_q4 - q4_m))/10 - (253*cos(omega_q1 - q1_m)*cos(omega_q4 - q4_m)*sin(omega_q4 - q4_m))/5, 0, -1/2, 0;
    (253*cos(omega_q1 - q1_m)*sin(omega_q3 - q3_m)*cos(omega_q4 - q4_m)^2)/20 + (161*cos(omega_q1 - q1_m)*sin(omega_q3 - q3_m)*cos(omega_q4 - q4_m))/10 - (253*cos(omega_q1 - q1_m)*sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m)^2)/20, 0, (161*sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m))/10 + (253*cos(omega_q4 - q4_m)*(sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m) + cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m)*sin(omega_q1 - q1_m)))/20 + (253*sin(omega_q4 - q4_m)*(cos(omega_q4 - q4_m)*sin(omega_q3 - q3_m) - cos(omega_q3 - q3_m)*sin(omega_q1 - q1_m)*sin(omega_q4 - q4_m)))/20 + (161*cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m)*sin(omega_q1 - q1_m))/10, (253*sin(omega_q4 - q4_m)*(cos(omega_q3 - q3_m)*sin(omega_q4 - q4_m) - cos(omega_q4 - q4_m)*sin(omega_q1 - q1_m)*sin(omega_q3 - q3_m)))/10 - (253*cos(omega_q4 - q4_m)*(cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m) + sin(omega_q1 - q1_m)*sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m)))/10 - (161*cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m))/10 - (161*sin(omega_q1 - q1_m)*sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m))/10, 0, 0, -1/2;
    (253*cos(omega_q1 - q1_m)*cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m)^2)/20 + (161*cos(omega_q1 - q1_m)*cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m))/10 - (253*cos(omega_q1 - q1_m)*cos(omega_q3 - q3_m)*sin(omega_q4 - q4_m)^2)/20, 0, (161*cos(omega_q3 - q3_m)*sin(omega_q4 - q4_m))/10 + (253*cos(omega_q4 - q4_m)*(cos(omega_q3 - q3_m)*sin(omega_q4 - q4_m) - cos(omega_q4 - q4_m)*sin(omega_q1 - q1_m)*sin(omega_q3 - q3_m)))/20 + (253*sin(omega_q4 - q4_m)*(cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m) + sin(omega_q1 - q1_m)*sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m)))/20 - (161*cos(omega_q4 - q4_m)*sin(omega_q1 - q1_m)*sin(omega_q3 - q3_m))/10, (161*cos(omega_q4 - q4_m)*sin(omega_q3 - q3_m))/10 + (253*cos(omega_q4 - q4_m)*(cos(omega_q4 - q4_m)*sin(omega_q3 - q3_m) - cos(omega_q3 - q3_m)*sin(omega_q1 - q1_m)*sin(omega_q4 - q4_m)))/10 - (253*sin(omega_q4 - q4_m)*(sin(omega_q3 - q3_m)*sin(omega_q4 - q4_m) + cos(omega_q3 - q3_m)*cos(omega_q4 - q4_m)*sin(omega_q1 - q1_m)))/10 - (161*cos(omega_q3 - q3_m)*sin(omega_q1 - q1_m)*sin(omega_q4 - q4_m))/10, -1/2, 0, 0];



[Phi, Gamma] = c2d(A, G, Ts);
end

function H = funcLinH()
H=eye(3);
end



