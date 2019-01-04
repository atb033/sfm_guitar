close all; clear; clc;

%% Initialization
addpath(genpath('./devkit'))
base_dir = '../data/2011_09_26_drive_0005_sync';
% calib_dir = './2011_09_26';

%% Read data
% Read vehicle Pose
oxts = loadOxtsliteData(base_dir);
pose = convertOxtsToPose(oxts);
% Read tracklets
tracklets = readTracklets([base_dir '/tracklet_labels.xml']); % slow version

%% Occupancy Grid Mapping
omap = robotics.OccupancyMap3D(1.5);
for i = 1:length(pose)
    sensor_pose = getSensorPose(i, pose); % car pose
    ptCloud = getPointCloud(i, base_dir); % point cloud
    
    insertPointCloud(omap, sensor_pose, ptCloud,  120); % Update Occupancy Grid

    % load and display scene image
    subplot(2,2,1:2) 
    cam = 2;
    img = imread(sprintf('%s/image_%02d/data/%010d.png',base_dir,cam,i-1));
    imshow(img);
    title('Scene Image')
    
    % display real time point cloud
    subplot(2,2,3) 
    pcshow(ptCloud)
    hold on
    set(gca, 'View', [0, 90])
    xlim([-60,60])
    ylim([-60,60])
    title('Instantaneous Point Cloud')
    l=4;b=1.5; h = 1.5;
    
    plot3([-l/2,l/2,l/2,-l/2,-l/2],[-b/2,-b/2,b/2,b/2,-b/2],[0,0,0,0,0], ...
        'b','LineWidth',1);
    xlabel('X[meters]')
    ylabel('Y[meters]')
    hold off
    pbaspect([1,1,1])
    
    % Display current occupancy grid map
    subplot(2,2,4)
    show(omap); % Display occupancy grid
    hold on;
    drawTrackletBoxes(i, tracklets, pose); % Display tracklets
    drawPose(i, pose); % Display path and car
    set(gca, 'View', [0, 90])
    hold off;
    pbaspect([1,1,1])
    pause(0.1);
    F(i) = getframe(gcf);
end

%% Create the video 
writerObj = VideoWriter('occupancy_1.avi');
writerObj.FrameRate = 9.74;
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    writeVideo(writerObj, F(i));
end
% close the writer object
close(writerObj);

