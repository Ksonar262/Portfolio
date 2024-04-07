%% 

clear ; clear all; close all;
% Read data files
data = readtable("input_data_cubic.xlsx");

%%
%extract data from the table to make into required vectors
Ts=10;
t=data.TS;
y_k = [data.LFIN_x data.LFIN_y data.LFIN_z];
c_k = [data.q1 data.q2 data.q3 data.q4 data.LSHO_x data.LSHO_y data.LSHO_z];

%remove NaN values which represent occlusion in shoulder
rows_with_nan = any(isnan(c_k), 2);
c_k = c_k(~rows_with_nan, :);

rows_with_nan_yk = any(isnan(y_k), 2);
y_k = y_k(~rows_with_nan_yk, :);

%initialisation of matrices and variables
xhat_km1_km1 = [-1010.67 -970.791 690.5866];
P_km1_km1 = diag([50 50 50]);

Q = [
    0.0100   -0.0000   -0.0000    0.0000   -0.0000    0.0000   -0.0000;
   -0.0000    0.0100    0.0000   -0.0000    0.0000    0.0000   -0.0000;
   -0.0000    0.0000    0.0100   -0.0000    0.0000   -0.0000   -0.0000;
    0.0000   -0.0000   -0.0000    0.0100   -0.0001   -0.0000    0.0000;
   -0.0000    0.0000    0.0000   -0.0001    0.3175   -0.0452   -0.0047;
    0.0000    0.0000   -0.0000   -0.0000   -0.0452    0.0754   -0.0251;
   -0.0000   -0.0000   -0.0000    0.0000   -0.0047   -0.0251    0.0375
];

R = [
    0.4545, -0.0621, 0.2800;
   -0.0621,  0.2006, -0.1017;
    0.2800, -0.1017,  0.3785
];

t=t(1:length(c_k), :);
y_k=y_k(1:length(c_k), :);

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

square_dif = sqrt((x_cor - y_k).^2);
mse = mean(square_dif,"omitnan");

%%


% Specify the filename for saving
filename = 'xcor_data.mat';

% Save the variable xcor to the specified filename
save(filename, 'x_cor');

%%
% Create figure 1 with 3 subplots
figure('Units', 'centimeters', 'Position', [2, 2, 12, 14]);
subplot(3,1,1);
plot(t, y_k(:,1),'LineWidth',2);
hold on;
plot(t, x_cor(:,1),'r--','LineWidth',3); % Changed to dashed line and reduced thickness
xlabel('t(s)','FontSize', 14);
ylabel('Actual H_x, Predicted H_x(m)','FontSize', 14);
legend('Actual', 'Predicted','FontSize', 14);
title('x', 'FontSize', 14);

subplot(3,1,2);
plot(t, y_k(:,2),'LineWidth',2);
hold on;
plot(t, x_cor(:,2),'r--','LineWidth',3); % Changed to dashed line and reduced thickness
xlabel('t(s)','FontSize', 14);
ylabel('Actual H_y, Predicted H_y(m)','FontSize', 14);
legend('Actual', 'Predicted','FontSize', 14);
title('y', 'FontSize', 14);

subplot(3,1,3);
plot(t, y_k(:,3),'LineWidth',2);
hold on;
plot(t, x_cor(:,3),'r--','LineWidth',3); % Changed to dashed line and reduced thickness
xlabel('t(s)','FontSize', 14);
ylabel('Actual H_z, Predicted H_z(m)','FontSize', 14);
legend('Actual', 'Predicted','FontSize', 14);
title('z', 'FontSize', 14);

set(gcf, 'PaperSize', [12, 14]);
set(gcf, 'PaperPosition', [0, 0, 12, 14]);
set(findobj(gcf,'type','axes'),'FontName','Arial','FontSize',14,'FontWeight','Normal', 'LineWidth', 2);
saveas(gcf,'Kalman Filter predictions.png')


figure('Units', 'centimeters', 'Position', [1, 1, 10, 6]);
hold on;
for col = 1:3
    plot(1:N, innov(:, col), 'Color', rand(1,3), 'DisplayName', ['Column ' num2str(col)],'LineWidth',2);
end
hold off;
xlabel('Time Step');
ylabel('Innovation');
title('Plot of Innovations in Kalman Filter');
legend('Location', 'best');
grid on;
set(gcf, 'PaperSize', [10, 6]);
set(gcf, 'PaperPosition', [0, 0, 10, 6]);
set(findobj(gcf,'type','axes'),'FontName','Arial','FontSize',14,'FontWeight','Normal', 'LineWidth', 2);
saveas(gcf,'kalman filter innovation.png')


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
omega_q1 = 0.2;
omega_q2 = 0.2;
omega_q3 = 0.2;
omega_q4 = 0.2;
Ts = 10;

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



