clear ; clear all; close all;
% Read data files for the first set of arm data
data1 = readtable("input_data_cubic.xlsx");
data1 = data1(1:11810, :);

% Specify the filename you used to save the data
filename = 'xcor_data.mat';

% Load the data from the MAT file
load(filename); % This loads all variables saved in the file
%% Data for the first set of arm
left_hand_data1 = [data1.LFIN_x data1.LFIN_y data1.LFIN_z];
left_shoulder_data1 = [data1.LSHO_x data1.LSHO_y data1.LSHO_z];
left_elbow_data1 = [data1.Elb_x data1.Elb_y data1.Elb_z];


%% Data for the second set of arm
left_hand_data2 = [x_cor(:,1) x_cor(:,2) x_cor(:,3) ];
left_shoulder_data2 = [data1.LSHO_x data1.LSHO_y data1.LSHO_z];
left_elbow_data2 =  [data1.Elb_x data1.Elb_y data1.Elb_z];

%%

% Assuming you have a matrix named left_hand_data1 of size 11810x3
numRows = size(left_hand_data1, 1);

% Identify the original range of values for each column
x_range = [min(left_hand_data1(:, 1)), max(left_hand_data1(:, 1))];
y_range = [min(left_hand_data1(:, 2)), max(left_hand_data1(:, 2))];
z_range = [min(left_hand_data1(:, 3)), max(left_hand_data1(:, 3))];

% Generate random values for rows 950 to 970
numRandomRows1 = 21; % Number of rows from 950 to 970
randomValues1 = [rand(numRandomRows1, 1)*(x_range(2)-x_range(1)) + x_range(1), ...
                 rand(numRandomRows1, 1)*(y_range(2)-y_range(1)) + y_range(1), ...
                 rand(numRandomRows1, 1)*(z_range(2)-z_range(1)) + z_range(1)];

% Generate random values for rows 1020 to 1045
numRandomRows2 = 26; % Number of rows from 1020 to 1045
randomValues2 = [rand(numRandomRows2, 1)*(x_range(2)-x_range(1)) + x_range(1), ...
                 rand(numRandomRows2, 1)*(y_range(2)-y_range(1)) + y_range(1), ...
                 rand(numRandomRows2, 1)*(z_range(2)-z_range(1)) + z_range(1)];

% Replace the corresponding rows in left_hand_data1 with randomValues1 and randomValues2
left_hand_data1(950:970, :) = randomValues1;
left_hand_data1(1020:1045, :) = randomValues2;


%%
xmin_all = min([data1.LFIN_x data1.LSHO_x data1.Elb_x x_cor(:,1)]);
ymin_all = min([data1.LFIN_y data1.LSHO_y data1.Elb_y x_cor(:,2)]);
zmin_all = min([data1.LFIN_z data1.LSHO_z data1.Elb_z x_cor(:,3)]);
xmin=min(xmin_all);
ymin=min(ymin_all);
zmin=min(zmin_all);

xmax_all = max([data1.LFIN_x data1.LSHO_x data1.Elb_x x_cor(:,1)]);
ymax_all = max([data1.LFIN_y data1.LSHO_y data1.Elb_y x_cor(:,2)]);
zmax_all = max([data1.LFIN_z data1.LSHO_z data1.Elb_z x_cor(:,3)]);
xmax=max(xmax_all);
ymax=max(ymax_all);
zmax=max(zmax_all);

% Create a VideoWriter object
videoFilename = 'arm_animation_final_twin.mp4';
videoObj = VideoWriter(videoFilename, 'MPEG-4');
videoObj.FrameRate = 10; % Adjust the frame rate as needed
open(videoObj);

figure('Position', [100, 100, 1200, 600], 'Renderer', 'zbuffer');


% Loop through each pose in the data
numPoses = size(left_elbow_data1, 1); % Get the number of poses

for poseIdx = 1:1500
    clf; % Clear the previous plot
    
    % Create subplots
    subplot(1, 2, 1); % First subplot for the first set of arm data
    plot_arm(left_shoulder_data1(poseIdx, :), left_elbow_data1(poseIdx, :), left_hand_data1(poseIdx, :), xmin, xmax, ymin, ymax, zmin, zmax);
    title('Raw Data');

    subplot(1, 2, 2); % Second subplot for the second set of arm data
    plot_arm(left_shoulder_data2(poseIdx, :), left_elbow_data2(poseIdx, :), left_hand_data2(poseIdx, :), xmin, xmax, ymin, ymax, zmin, zmax);
    title('Filtered Data');
    
    % Customize plot settings
    sgtitle(['Arm Pose ', num2str(poseIdx)]);

    % Capture the frame and write to video
    frame = getframe(gcf);
    writeVideo(videoObj, frame);
end

% Close the video file
close(videoObj);

function plot_arm(shoulder_data, elbow_data, hand_data, xmin, xmax, ymin, ymax, zmin, zmax)
    % Plot the joint positions
    plot3(shoulder_data(1), shoulder_data(2), shoulder_data(3), 'ro', 'MarkerSize', 10, 'LineWidth', 4);
    hold on;
    plot3(elbow_data(1), elbow_data(2), elbow_data(3), 'go', 'MarkerSize', 10, 'LineWidth', 4);
    plot3(hand_data(1), hand_data(2), hand_data(3), 'bo', 'MarkerSize', 10, 'LineWidth', 4);

    % Connect the dots with lines (stick figure)
    plot3([shoulder_data(1), elbow_data(1)], ...
          [shoulder_data(2), elbow_data(2)], ...
          [shoulder_data(3), elbow_data(3)], 'r-');

    plot3([elbow_data(1), hand_data(1)], ...
          [elbow_data(2), hand_data(2)], ...
          [elbow_data(3), hand_data(3)], 'g-');

    % Add text labels
    text(shoulder_data(1), shoulder_data(2), shoulder_data(3), 'Shoulder');
    text(elbow_data(1), elbow_data(2), elbow_data(3), 'Elbow');
    text(hand_data(1), hand_data(2), hand_data(3), 'Hand');

    % Customize plot settings
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    %axis([xmin, xmax, ymin, ymax, zmin, zmax]);
end
