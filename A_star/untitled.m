close all;
% Read input image
inputImage = imread('map.bmp');  % Change the filename to your input image

% Convert the image to grayscale if it is in color
if size(inputImage, 3) == 3
    grayImage = rgb2gray(inputImage);
else
    grayImage = inputImage;
end

% Determine start and end points interactively
figure;
imshow(grayImage);
title('Click on the Start and Target Points');

% Let the user select the start point
[xStart, yStart] = ginput(1);
xStart = round(xStart);
yStart = round(yStart);

% Mark the start point on the image
hold on;
plot(xStart, yStart, 'bo', 'MarkerSize', 10);
text(xStart + 1, yStart + 1, 'Start');

% Let the user select the target point
[xTarget, yTarget] = ginput(1);
xTarget = round(xTarget);
yTarget = round(yTarget);

% Mark the target point on the image
hold on;
plot(xTarget, yTarget, 'gd', 'MarkerSize', 10);
text(xTarget + 1, yTarget + 1, 'Target');

% Determine obstacles based on intensity threshold
%intensityThreshold = 128;  % Adjust the threshold based on your image
binaryMap = 3*grayImage - 1;

% Create binary map
%binaryMap = ~binaryMap;

% Initialize map
MAX_X = size(binaryMap, 2);
MAX_Y = size(binaryMap, 1);
MAP = binaryMap;

% ... (Rest of the code remains unchanged)


% ... (Rest of the code remains unchanged)

%End of obstacle-Target pickup
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
% Assume you have the MAP matrix defined with obstacles (-1), open spaces (2), start (1), and target (0)

% Lists used for the algorithm
OPEN = [];
CLOSED = [];

% Put all obstacles on the Closed list
[k, l] = find(MAP == -1);
CLOSED = [k, l];

CLOSED_COUNT = size(CLOSED, 1);

% Set the starting node as the first node
xNode = xStart;  % Assuming xStart is defined earlier
yNode = yStart;  % Assuming yStart is defined earlier

OPEN_COUNT = 1;
path_cost = 0;
goal_distance = distance(xNode, yNode, xTarget, yTarget);
OPEN(OPEN_COUNT, :) = insert_open(xNode, yNode, xNode, yNode, path_cost, goal_distance, goal_distance);
OPEN(OPEN_COUNT, 1) = 0;
CLOSED_COUNT = CLOSED_COUNT + 1;
CLOSED(CLOSED_COUNT, 1) = xNode;
CLOSED(CLOSED_COUNT, 2) = yNode;

NoPath = 1;

% Start A* algorithm
while ((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
    exp_array = expand_array(xNode, yNode, path_cost, xTarget, yTarget, CLOSED, MAX_X, MAX_Y);
    exp_count = size(exp_array, 1);

    % Update list OPEN with the successor nodes
    for i = 1:exp_count
        flag = 0;
        for j = 1:OPEN_COUNT
            if (exp_array(i, 1) == OPEN(j, 2) && exp_array(i, 2) == OPEN(j, 3))
                OPEN(j, 8) = min(OPEN(j, 8), exp_array(i, 5));
                if OPEN(j, 8) == exp_array(i, 5)
                    % Update parents, gn, hn
                    OPEN(j, 4) = xNode;
                    OPEN(j, 5) = yNode;
                    OPEN(j, 6) = exp_array(i, 3);
                    OPEN(j, 7) = exp_array(i, 4);
                end
                flag = 1;
            end
        end

        if flag == 0
            OPEN_COUNT = OPEN_COUNT + 1;
            OPEN(OPEN_COUNT, :) = insert_open(exp_array(i, 1), exp_array(i, 2), xNode, yNode, exp_array(i, 3), exp_array(i, 4), exp_array(i, 5));
        end
    end

    % Find the node with the smallest fn
    index_min_node = min_fn(OPEN, OPEN_COUNT, xTarget, yTarget);
    
    if (index_min_node ~= -1)
        % Set xNode and yNode to the node with minimum fn
        xNode = OPEN(index_min_node, 2);
        yNode = OPEN(index_min_node, 3);
        path_cost = OPEN(index_min_node, 6);
        
        % Move the Node to list CLOSED
        CLOSED_COUNT = CLOSED_COUNT + 1;
        CLOSED(CLOSED_COUNT, 1) = xNode;
        CLOSED(CLOSED_COUNT, 2) = yNode;
        OPEN(index_min_node, 1) = 0;
    else
        % No path exists to the Target!!
        NoPath = 0;
    end
end

% Generate optimal path
i = size(CLOSED, 1);
Optimal_path = [];
xval = CLOSED(i, 1);
yval = CLOSED(i, 2);
i = 1;
Optimal_path(i, 1) = xval;
Optimal_path(i, 2) = yval;
i = i + 1;

if ((xval == xTarget) && (yval == yTarget))
    inode = 0;
    
    % Traverse OPEN and determine the parent nodes
    parent_x = OPEN(node_index(OPEN, xval, yval), 4);
    parent_y = OPEN(node_index(OPEN, xval, yval), 5);

    while (parent_x ~= xStart || parent_y ~= yStart)
        Optimal_path(i, 1) = parent_x;
        Optimal_path(i, 2) = parent_y;
        
        % Get the grandparents
        inode = node_index(OPEN, parent_x, parent_y);
        parent_x = OPEN(inode, 4);
        parent_y = OPEN(inode, 5);
        i = i + 1;
    end
    
    % Plot the Optimal Path
    j = size(Optimal_path, 1);
    p = plot(Optimal_path(j, 1) + 0.5, Optimal_path(j, 2) + 0.5, 'bo');
    j = j - 1;

    for i = j:-1:1
        pause(0.25);
        set(p, 'XData', Optimal_path(i, 1) + 0.5, 'YData', Optimal_path(i, 2) + 0.5);
        drawnow;
    end

    plot(Optimal_path(:, 1) + 0.5, Optimal_path(:, 2) + 0.5);
else
    pause(1);
    h = msgbox('Sorry, No path exists to the Target!', 'warn');
    uiwait(h, 5);
end