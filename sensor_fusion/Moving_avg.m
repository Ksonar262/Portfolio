% Example hand position data (replace this with your actual data)
data = readtable("D:\Dissertation\Kalman Filtering\input_data_cubic.xlsx");

hand_positions=[data.LFIN_x data.LFIN_y data.LFIN_z];

% Calculate the 100-point moving average
displacement = diff(hand_positions);
speeds = displacement ./ 10; % Note: We exclude the first time interval
speeds = abs(speeds);

mean_speed = mean(speeds, "omitnan");


% Create a logical index for NaN values
%nan_indices = isnan(speeds);

% Replace NaN values with 0
%speeds(nan_indices) = 0;

%S=sum(speeds);
%L=length(speeds);
%avg_speed=S/L;

%window_size = 100;
%moving_average = movmean(speeds, window_size);
%mode_speed=mode(moving_average);
%min(moving_average)
%max(moving_average)

%plot(hand_positions, 'b');         % Original data in blue
%hold on;
%plot(moving_average, 'r', 'LineWidth', 2); % Moving average in red
%legend('Moving Average');
%xlabel('Time');
%ylabel('Hand Position');
%title('100-Point Moving Average of Hand Position');
