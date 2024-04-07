% Given parameters
angles_and_position = [1.568991269	-0.002756889	-0.004760252	0.086172566 -1199.755493 -1014.23645 1098.99646];

% Call the function to calculate hand coordinates
[H_x, H_y, H_z] = humanArmForwardKinematics(angles_and_position);

% Display hand coordinates
fprintf('Hand coordinates:\n');
fprintf('H_x: %.2f\nH_y: %.2f\nH_z: %.2f\n', H_x, H_y, H_z);
