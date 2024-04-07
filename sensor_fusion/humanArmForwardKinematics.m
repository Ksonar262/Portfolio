function [H_x, H_y, H_z] = humanArmForwardKinematics(angles_and_position)
    alpha = angles_and_position(3);   % Shoulder roll angle in radians
    gamma = angles_and_position(1);   % Shoulder yaw angle in radians
    theta1 = angles_and_position(2);  % Shoulder pitch angle in radians
    theta2 = angles_and_position(4);  % Elbow pitch angle in radians
    S = angles_and_position(5:7);     % Shoulder position (x, y, z)
    
    L1 = 32.2;
    L2 = 25.3;
    
    Rx = @(alpha) [1, 0, 0;
                   0, cos(alpha), -sin(alpha);
                   0, sin(alpha), cos(alpha)];
    Ry = @(gamma) [cos(gamma), 0, sin(gamma);
                   0, 1, 0;
                   -sin(gamma), 0, cos(gamma)];
    Rz = @(theta) [cos(theta), -sin(theta), 0;
                   sin(theta), cos(theta), 0;
                   0, 0, 1];
    T_S_pre = [Rx(alpha) * Ry(gamma) * Rz(theta1)]

    T_S = [Rx(alpha) * Ry(gamma) * Rz(theta1), S.'; 0 0 0 1];
    T_SE = [Rz(theta2), [L1; 0; 0]; 0 0 0 1];
    T_EH = [eye(3), [L2; 0; 0]; 0 0 0 1];
    
    T_SH = T_S * T_SE * T_EH;
    
    H_x = T_SH(1, 4);
    H_y = T_SH(2, 4);
    H_z = T_SH(3, 4);
end