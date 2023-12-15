% Read the image file
imagePath = 'path_to_your_image.png';  % Replace with the path to your image
image = imread('ss.png');

% Convert the image to a binary occupancy map
threshold = 0;  % Adjust the threshold based on your image
binaryImage = im2bw(image, threshold/255);
mapData = binaryOccupancyMap(binaryImage, 1);

% Rest of the code remains the same
startPose = [100 100 pi/2];
goalPose = [400 410 pi/2];
path = codegenPathPlanner(mapData, startPose, goalPose);

% Display the map and path
show(mapData)
hold on
scatter(startPose(1,1), startPose(1,2), "g", "filled")
scatter(goalPose(1,1), goalPose(1,2), "r", "filled")
plot(path(:,1), path(:,2), "r-", 'LineWidth', 2)
legend("Start Pose", "Goal Pose", "MATLAB Generated Path")
legend(Location="northwest")
