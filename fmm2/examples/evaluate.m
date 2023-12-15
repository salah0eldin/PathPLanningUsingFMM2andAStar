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

    % Plotting map
    figure(2);
    clf(2); % Clear figure 2 to avoid overlap
    subplot(1, 3, 1); % Subplot 1
    imagesc(map);
    colormap gray(256);
    axis xy;
    axis image;
    axis off;
    hold on;
    plot(start(1), start(2), 'rx', 'MarkerSize', 15);
    plot(goal(1), goal(2), 'k*', 'MarkerSize', 15);

    %% Fast Marching Square
    startTime = tic;
    [F, T, path, vels, times] = FM2(map, sat, start, goal);
    endTime = toc(startTime);

    % Updating map
    subplot(1, 3, 2); % Subplot 2
    plot(path(1,:), path(2,:), 'b-', 'LineWidth', 3);

    % Plotting velocities map.
    subplot(1, 3, 3); % Subplot 3
    imagesc(F);
    colormap gray(256);
    axis xy;
    axis image;
    axis off;

    % Showing times info.
    str = sprintf('Time for F map: %f\nTime for T map: %f.\nEvaluation time: %f.',times(1),times(2),endTime);
    disp(str);
    % Function to calculate path length
    disp(pathLength(path));
end
