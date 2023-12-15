% Specify the size of the image
imageSize = [1000, 1000]; % Adjust the size as needed

% Create a white image (all ones)
whiteImage = ones(imageSize);

% Display the image
hFig = figure;
imshow(whiteImage, 'InitialMagnification', 'fit');
title('Draw obstacles using the mouse');

% Allow the user to draw multiple obstacles using the mouse
obstacleMasks = [];
while true
    % Let the user draw an obstacle using the mouse
    hPoly = impoly;

    % Wait for the user to finish drawing and get the mask
    wait(hPoly);
    obstacleMask = createMask(hPoly);
    
    % Add the obstacle mask to the list
    obstacleMasks = cat(3, obstacleMasks, obstacleMask);

    % Ask the user if they want to draw another obstacle
    choice = questdlg('Do you want to draw another obstacle?', 'Draw Another Obstacle', 'Yes', 'No', 'No');
    if strcmp(choice, 'No')
        break;
    end
end

% Close the figure
close(hFig);

% Set the obstacle pixels in the white image based on all the masks
whiteImage(any(obstacleMasks, 3)) = 0; % Set obstacle pixels to black

% Display the final image
imshow(whiteImage, 'InitialMagnification', 'fit');

% Save the image to a file (optional)
imwrite(whiteImage, 'test4.png');
