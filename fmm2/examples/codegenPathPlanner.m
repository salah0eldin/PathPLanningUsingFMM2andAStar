function path = codegenPathPlanner(mapData,startPose,goalPose)
    %#codegen
    
    % Create a binary occupancy map
    map = binaryOccupancyMap(mapData);

    % Create a state space object
    stateSpace = stateSpaceSE2;

    % Update state space bounds to be the same as map limits.
    stateSpace.StateBounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];

    % Construct a state validator object using the statespace and map object
    validator = validatorOccupancyMap(stateSpace, map);
    
    % Set the validation distance for the validator
    validator = validatorOccupancyMap(stateSpace, map);

    
    % Assign the state validator object to the plannerHybridAStar object
    planner = plannerHybridAStar(validator);
    
    % Create a figure for plotting
    figure;
    show(map);
    hold on;
    scatter(startPose(1,1), startPose(1,2), "g", "filled");
    scatter(goalPose(1,1), goalPose(1,2), "r", "filled");
    
    % Initialize an empty path
    path = [];
    
    % Compute a path for the given start and goal poses
    pathObj = plan(planner, startPose, goalPose);
    
    % Extract the path poses from the path object
    path = pathObj.States;

    % Plot the path as it is being generated
    for i = 1:size(path, 1)
        plot(path(1:i, 1), path(1:i, 2), "b-", 'LineWidth', 1.5);
        drawnow;
    end

    % Add the final path to the plot
    plot(path(:, 1), path(:, 2), "r-", 'LineWidth', 2);

    % Add legend
    legend("Start Pose", "Goal Pose", "Path (Partial)", "Final Path");
    legend(Location="northwest");
end
