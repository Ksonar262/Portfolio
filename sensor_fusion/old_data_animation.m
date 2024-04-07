clear ; clear all; close all;
% Read data files
data = readtable("participant1_position_orientation_left_fa_segment.xlsx");

%%

left_hand_data = [data.X_ha data.Y_ha data.Z_ha];
left_shoulder_data = [data.X_ua data.Y_ua data.Z_ua];
left_elbow_data = [data.X_fa data.Y_fa data.Z_fa];
torso_data = zeros(height(data), 3);

%data.X_ha(100:500, :) = repmat(1000, length(100:500), 1);

time_steps = data.TS;

xmin_all = min([data.X_ha data.X_fa data.X_ua]);
ymin_all = min([data.Y_ha data.Y_fa data.Y_ua]);
zmin_all = min([data.Z_ha data.Z_fa data.Z_ua]);
xmin=min(xmin_all);
ymin=min(ymin_all);
zmin=min(zmin_all);

xmax_all = max([data.X_ha data.X_fa data.X_ua]);
ymax_all = max([data.Y_ha data.Y_fa data.Y_ua]);
zmax_all = max([data.Z_ha data.Z_fa data.Z_ua]);
xmax=max(xmax_all);
ymax=max(ymax_all);
zmax=max(zmax_all);

% Create a VideoWriter object
videoFilename = 'arm_animation_raw.mp4';
videoObj = VideoWriter(videoFilename, 'MPEG-4');
videoObj.FrameRate = 10; % Adjust the frame rate as needed
open(videoObj);

% Loop through each pose in the data
numPoses = size(left_elbow_data, 1); % Get the number of poses

for poseIdx = 1:numPoses
    clf; % Clear the previous plot
    
    % Plot the joint positions
    plot3(left_shoulder_data(poseIdx, 1), left_shoulder_data(poseIdx, 2), left_shoulder_data(poseIdx, 3), 'ro', 'MarkerSize', 10);
    hold on;
    plot3(left_elbow_data(poseIdx, 1), left_elbow_data(poseIdx, 2), left_elbow_data(poseIdx, 3), 'go', 'MarkerSize', 10);
    plot3(left_hand_data(poseIdx, 1), left_hand_data(poseIdx, 2), left_hand_data(poseIdx, 3), 'bo', 'MarkerSize', 10);
    
    % Connect the dots with lines (stick figure)
    plot3([left_shoulder_data(poseIdx, 1), left_elbow_data(poseIdx, 1)], ...
          [left_shoulder_data(poseIdx, 2), left_elbow_data(poseIdx, 2)], ...
          [left_shoulder_data(poseIdx, 3), left_elbow_data(poseIdx, 3)], 'r-');
    
    plot3([left_elbow_data(poseIdx, 1), left_hand_data(poseIdx, 1)], ...
          [left_elbow_data(poseIdx, 2), left_hand_data(poseIdx, 2)], ...
          [left_elbow_data(poseIdx, 3), left_hand_data(poseIdx, 3)], 'g-');
    
    % Add text labels
    text(left_shoulder_data(poseIdx, 1), left_shoulder_data(poseIdx, 2), left_shoulder_data(poseIdx, 3), 'Shoulder');
    text(left_elbow_data(poseIdx, 1), left_elbow_data(poseIdx, 2), left_elbow_data(poseIdx, 3), 'Elbow');
    text(left_hand_data(poseIdx, 1), left_hand_data(poseIdx, 2), left_hand_data(poseIdx, 3), 'Hand');
    
    % Customize plot settings
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title(['Arm Pose ', num2str(poseIdx)]);
    
    %Set axis limits if needed
    axis([xmin, xmax, ymin, ymax, zmin, zmax]);
    
    % Set the desired view angle (adjust as needed)
    view(85, 20); % Azimuth -30, Elevation 20
    
    drawnow; % Update the plot

    % Print progress
    fprintf('Rendering frame %d of %d\n', poseIdx, numPoses);
    
    % Capture the frame and write to video
    frame = getframe(gcf);
    writeVideo(videoObj, frame);
end

% Close the video file
close(videoObj);
