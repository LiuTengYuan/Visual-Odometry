clear; close all; clc;
%% Correct Fisheye Image for Lens Distortion
% Remove lens distortion from a fisheye image by detecting a 
% checkboard calibration pattern and calibrating the camera. Then, display the results.

%%
% Gather a set of checkerboard calibration images.
images = imageDatastore(fullfile(toolboxdir('vision'),'visiondata', ...
    'calibration','gopro'));
%%
% Detect the calibration pattern from the images.
[imagePoints,boardSize] = detectCheckerboardPoints(images.Files);

%%
% Generate world coordinates for the corners of the checkerboard squares.
squareSize = 29; % millimeters
worldPoints = generateCheckerboardPoints(boardSize,squareSize);

%%
% Estimate the fisheye camera calibration parameters based on the image and
% world points. Use the first image to get the image size.
I = readimage(images,1); 
imageSize = [size(I,1) size(I,2)];
params = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);

%%
% % Remove lens distortion from the first image |I| and display the results.
% J1 = undistortFisheyeImage(I,params.Intrinsics);
% figure
% imshowpair(I,J1,'montage')
% title('Original Image (left) vs. Corrected Image (right)')
% 
% J2 = undistortFisheyeImage(I,params.Intrinsics,'OutputView','full');
% figure
% imshow(J2)
% title('Full Output View')
%%
% %Visualize the calibration accuracy.
% showReprojectionErrors(params);
% 
% %Visualize camera extrinsics.
% figure;
% showExtrinsics(params);
% 
% %Plot detected and reprojected points
% figure; 
% imshow(images.Files{1}); 
% hold on;
% plot(imagePoints(:,1,1), imagePoints(:,2,1),'go');
% plot(params.ReprojectedPoints(:,1,1),params.ReprojectedPoints(:,2,1),'r+');
% legend('Detected Points','ReprojectedPoints');
% hold off;

