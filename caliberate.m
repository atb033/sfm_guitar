
numimages = 4;
files = cell(1,numimages);

for i=1:4
    files{i} = sprintf('%s/chess%01d.jpg',dir,i);
%     files{i} = fullfile(toolboxdir('vision'),...
%            'visiondata', 'calibration', 'icam', filename);
end
 
% imshow(files{2})
%% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

%% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 20; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

%% Calibrate the camera.
imageSize = [size(files, 1), size(files, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);
                                 
%%
% Evaluate calibration accuracy.
figure; showReprojectionErrors(cameraParams);
title('Reprojection Errors');