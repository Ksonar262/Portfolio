% Define symbolic variables
syms alpha gamma theta1 theta2 S_x S_y S_z L1 L2

Rx = [1, 0, 0;
      0, cos(alpha), -sin(alpha);
      0, sin(alpha), cos(alpha)];

Ry = [cos(gamma), 0, sin(gamma);
      0, 1, 0;
      -sin(gamma), 0, cos(gamma)];

Rz = [cos(theta1 + theta2), -sin(theta1 + theta2), 0;
      sin(theta1 + theta2), cos(theta1 + theta2), 0;
      0, 0, 1];

T_S = [Rx * Ry * Rz, [S_x; S_y; S_z]; 0 0 0 1];
T_SE = [Rz, [L1; 0; 0]; 0 0 0 1];
T_EH = [eye(3), [L2; 0; 0]; 0 0 0 1];

T_SH = T_S * T_SE * T_EH