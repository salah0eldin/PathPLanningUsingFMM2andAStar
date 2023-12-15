function pathPlanningApp
originalMap = [];
% Create the main figure
mainFig = figure('Name', 'Path Planning App', 'Position', [100, 100, 800, 600]);

% Create UI components
uicontrol('Style', 'text', 'String', 'Saturation:', 'Position', [20, 550, 80, 20]);
satEdit = uicontrol('Style', 'edit', 'String', '0.01', 'Position', [100, 550, 80, 20]);

loadMapButton = uicontrol('Style', 'pushbutton', 'String', 'Load Map', 'Position', [200, 550, 80, 20], 'Callback', @loadMap);

uicontrol('Style', 'text', 'String', 'Start Point:', 'Position', [300, 550, 80, 20]);
startText = uicontrol('Style', 'edit', 'String', 'Not set', 'Position', [380, 550, 120, 20]);

uicontrol('Style', 'text', 'String', 'Goal Point:', 'Position', [520, 550, 80, 20]);
goalText = uicontrol('Style', 'edit', 'String', 'Not set', 'Position', [600, 550, 120, 20]);

evaluateButton = uicontrol('Style', 'pushbutton', 'String', 'Evaluate', 'Position', [730, 550, 80, 20], 'Callback', @evaluate);

% Map and axes for plotting
map = [];
mapAxes = axes('Parent', mainFig, 'Position', [0.05, 0.2, 0.6, 0.7]);
colormap(mapAxes, gray(256));
axis(mapAxes, 'xy', 'image', 'off');
hold(mapAxes, 'on');

% Callback function to load a map
    function loadMap(~, ~)
        [filename, pathname] = uigetfile({'*.png;*.jpg;*.bmp', 'Image Files (*.png, *.jpg, *.bmp)'; '*.*', 'All Files (*.*)'}, 'Select a Map');
        if isequal(filename, 0) || isequal(pathname, 0)
            return; % User canceled
        end

        % Load the map

        originalMap = imread(fullfile(pathname, filename));
        originalMap = flipdim(originalMap, 1);
        if size(originalMap, 3) == 3
            map = rgb2gray(originalMap);
        else
            map = originalMap;
        end
        % Display the map
        imagesc(map, 'Parent', mapAxes);
        axis(mapAxes, 'xy', 'image', 'off');
        title(mapAxes, 'Loaded Map');

        % Enable interactive point selection
        set(mainFig, 'WindowButtonDownFcn', @selectPoint);
    end

% Callback function to interactively select start and target points
    function selectPoint(~, ~)
        % Get the mouse click coordinates
        [x, y] = ginput(1);

        % Check if start point is set
        if strcmp(get(startText, 'String'), 'Not set')
            set(startText, 'String', sprintf('Start: (%.2f, %.2f)', x, y));
        else
            set(goalText, 'String', sprintf('Goal: (%.2f, %.2f)', x, y));

            % Disable further point selection
            set(mainFig, 'WindowButtonDownFcn', []);
        end
    end

% Callback function to evaluate
    function evaluate(~, ~)
        if isempty(map)
            msgbox('Load a map first!', 'Error', 'error');
            return;
        end

        % Get values from UI components
        sat = str2double(get(satEdit, 'String'));
        startStr = get(startText, 'String');
        goalStr = get(goalText, 'String');

        % Check if start and goal points are set
        if strcmp(startStr, 'Not set') || strcmp(goalStr, 'Not set')
            msgbox('Set both start and goal points!', 'Error', 'error');
            return;
        end

        % Extract start and goal coordinates from the strings
        start = sscanf(startStr, 'Start: (%f, %f)');
        goal = sscanf(goalStr, 'Goal: (%f, %f)');

        % Call your path planning function or do something with sat, start, and goal
        % if isempty(regexp(path,['algorithms' pathsep], 'once'))
        %     addpath([pwd, '/../algorithms']);    % path algorithms
        % end

        %   if isempty(regexp(path,['fm2tools' pathsep], 'once'))
        %      addpath([pwd, '/../fm2tools']);    % path algorithms
        % end

         % Apply a threshold to the grayscale map
        threshold = 128;
        binarymap = map < threshold;
        map(binarymap) = -1;
        % Plotting map
        %figure(1);
        figure(2);
        hold on;
        imagesc(originalMap);
        colormap gray(256);
        axis xy;
        axis image;
        axis off;
        plot(start(1), start(2), 'rx', 'MarkerSize', 15);
        plot(goal(1), goal(2), 'k*', 'MarkerSize', 15);


        %% Fast Marching Square
        startTime = tic;
        [F, T, path, vels, times] = FM2(map, sat, start, goal);
        endTime = toc(startTime);
        %%
        % Updating map
        figure(2)
        hold on
        plot(path(1,:), path(2,:), 'b-', 'LineWidth', 3);

        % Plotting velocities map.
        figure(3);
        imagesc(F);
        colormap gray(256);
        axis xy;
        axis image;
        axis off;

        % Plotting times-of-arrival map.
        figure(4)
        imagesc(T);
        colormap jet;
        axis xy;
        axis image;
        axis off;

        % Showing times info.
        str = sprintf('Time for F map: %f\nTime for T map: %f.\nEvaluation time: %f.',times(1),times(2),endTime);
        disp(str);
        % Function to calculate path length
        disp(pathLength(path));
    end
end
function lengthValue = pathLength(path)
dx = diff(path(1, :));
dy = diff(path(2, :));
lengthValue = sum(sqrt(dx.^2 + dy.^2));
end
