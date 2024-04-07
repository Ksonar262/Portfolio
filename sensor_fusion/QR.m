data = readtable("input_data_cubic.xlsx");

%extract data from the table to make into required vectors
Ts=10;
t=data.Time;
y_k = [data.LFIN_x data.LFIN_y data.LFIN_z];
c_k = [data.q1 data.q2 data.q3 data.q4 data.LSHO_x data.LSHO_y data.LSHO_z];
size_before = size(c_k);
% Find rows with NaN values
rows_with_nan = any(isnan(c_k), 2);

% Remove rows with NaN values
c_k = c_k(~rows_with_nan, :);
size_after=size(c_k);

% Find rows with NaN values
rows_with_nan = any(isnan(y_k), 2);

% Remove rows with NaN values
y_k = y_k(~rows_with_nan, :);
size_after=size(y_k);


% Calculate differences for process noise estimation
imu_differences = diff(c_k);
process_noise_covariance = cov(imu_differences); % Estimated Q matrix


% Calculate differences for process noise estimation
MD = diff(y_k);
measurement_cov = cov(MD); % Estimated Q matrix


% Display the estimated Q and R matrices
disp("Estimated Q Matrix (Process Noise Covariance):");
disp(process_noise_covariance);

disp(measurement_cov);

