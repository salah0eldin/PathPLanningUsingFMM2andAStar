close all;
[filename, pathname] = uigetfile({'*.png;*.jpg;*.bmp', 'Image Files (*.png, *.jpg, *.bmp)'; '*.*', 'All Files (*.*)'}, 'Select a Map');
if isequal(filename, 0) || isequal(pathname, 0)
    return; % User canceled
end

% Load the map

mapData = ~imread(fullfile(pathname, filename));
mem1 = memory;
show(binaryOccupancyMap(mapData))
hold on

% Use ginput to select start and goal points
disp('Select start point...')
startPoint = ginput(1);
startPose = [startPoint, pi/2];

disp('Select goal point...')
goalPoint = ginput(1);
goalPose = [goalPoint, pi/2];

% Record the start time
startTime = tic;

% Call the path planner
path = codegenPathPlanner(mapData, startPose, goalPose);

% Record the end time
endTime = toc(startTime);
mem2 = memory;
disp(mem2.MemUsedMATLAB - mem1.MemUsedMATLAB);

% Calculate the total length of the path
totalPathLength = 0;
for i = 2:size(path, 1)
    totalPathLength = totalPathLength + norm(path(i, 1:2) - path(i-1, 1:2));
end

% Start state
scatter(startPose(1), startPose(2), "g", "filled")
% Goal state
scatter(goalPose(1), goalPose(2), "r", "filled")
% Path
plot(path(:, 1), path(:, 2), "r-", 'LineWidth', 2)
legend("Start Pose", "Goal Pose", "MATLAB Generated Path")
legend('Location', 'northwest')

% Display the time and path length information
fprintf('Time taken: %.4f seconds\n', endTime);
fprintf('Total path length: %.4f units\n', totalPathLength);
