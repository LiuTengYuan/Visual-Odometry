% EXAMPLE of the computation projection of points for the fisheye camera models
% open fisheyeIntrinsics

%referenece
%https://www.mathworks.com/help/vision/ug/single-camera-calibrator-app.html

clear all
close all
clc

%% Image creation

% 3D points in homogeneous coordinate: planar grid
grid_size = [8,11]; % size of grid
grid_spacing = 0.005; % spacing between grid points
grid_origin = [-0.03 -0.023];
XYZ(1,:) = grid_origin(1) + repmat((1:grid_size(2))*grid_spacing, 1, grid_size(1));
XYZ(2,:) = grid_origin(2) + kron((1:grid_size(1))*grid_spacing, ones(1,grid_size(2)));
XYZ(3,:) = 0.1;
XYZ(4,:) = 1;

% % Randomly distributed points
% Npoints = 100;
% XYZ(1:2,:) = (rand(2,Npoints)-0.5)*0.2;
% XYZ(3,:) = rand(1,Npoints)*0.1;
% XYZ(4,:) = 1;

%camera positions & orientation
x0 = [-.02, .03,-.03, .03, .02,-.03, .02,-.03,   0,   0,   0];
y0 = [ .03, .04,-.01,-.01, .01, .01,   0,-.01, .02,   0,   0];
z0 = [   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0];
a = [   10,  20,   0,   0,   0,   0, -10,   0,  10,  10,  10];
b = [  -10,   0,  10,  10,   0,   0,  10,   0, -10,   0,  20];
c = [    0, -10,  10,   0,   0,   0,   0, -10, -10,  20, -10];
NumCamera = length(x0);

%camera calibration
boardSize = grid_size-1;
squareSize = grid_spacing*1000; % millimeters
worldPoints = generateCheckerboardPoints(boardSize,squareSize);
imageSize = [1280 720];
imagePoints = zeros(prod(boardSize-1),2,NumCamera);


%% Scenario definition
for ind_cam = 1:NumCamera
    % Fisheye Camera Parameters: C=[x0,y0,z0,a,b,c,a0,a2,a3,a4,cx,cy,s1,s2,s3,s4,Nu,Nv];
    % x0,y0,z0   : (1-3)   location (in meters)
    % a,b,c      : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % a0,a2,a3,a4: (7-10)  mapping coefficeients: Scaramuzza's Taylor model for projection function
    % cx,cy      : (11-12) center of distortion (in pixels)
    % s1,s2,s3,s4: (13-16) stretch matrix: 2-by-2 transformation matrix from sensor plane to camera image plane
    % Nu,Nv      : (17-18) Number of pixels
    
    Cfisheye=[x0(ind_cam),y0(ind_cam),z0(ind_cam),...%x0,y0,z0
        a(ind_cam)*pi/180,b(ind_cam)*pi/180,c(ind_cam)*pi/180,...%a,b,c
        1000.08,-1.02e-3,-3.82e-06,1.71e-10,...% a0,a2,a3,a4
        640,360,...% cx,cy
        1,0,0,1,...% s1,s2,s3,s4
        1280,720];% Nu,Nv
    
    % Camera Parameters: C=[x0,y0,z0,a,b,c,f,u0,v0,ku,kv];
    % x0,y0,z0: (1-3)   location (in meters)
    % a,b,c   : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % f       : (7)     focal length (in m)
    % u0,v0   : (8-9)   optical center (in pixels)
    % ku,kv   : (10-11) inverse of pixel size (in pixels/m)
    % Nu,Nv   : (12-13) Number of pixels
    
    C=[x0(ind_cam),y0(ind_cam),z0(ind_cam),...%x0,y0,z0
        a(ind_cam)*pi/180,b(ind_cam)*pi/180,c(ind_cam)*pi/180,...%a,b,c
        0.050,...% f
        1280/2,720/2,...% u0,v0
        1/23e-6,1/23e-6,...% ku,kv
        1280,720];% Nu,Nv
    
    
    %% Project points onto the image plane
    
    % Perspective Camera
    P = PerspectiveProjectionMatrix(C);
    UV = P*XYZ;
    for ind_points = 1:size(UV,2);
        UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
    end; clear ind_points
    
    % Fisheye Camera
    UV_fisheye = FisheyeDistortion(XYZ,Cfisheye,C);
    
    % MODEL: r=f*theda
    % UV = ones(3,size(XcYcZc,2));
    % for ind_points = 1:size(XcYcZc,2)
    %     r = sqrt(XcYcZc(1,ind_points)^2+XcYcZc(2,ind_points)^2);
    %     UV(1,ind_points) = C(8)+C(7)*C(10)*atan2(r,XcYcZc(3,ind_points))*XcYcZc(1,ind_points)/r;
    %     UV(2,ind_points) = C(9)+C(7)*C(11)*atan2(r,XcYcZc(3,ind_points))*XcYcZc(2,ind_points)/r;
    % end; clear ind_points
    
    % Exclude points outside the image
    isInsideImage = ( UV_fisheye(1,:) <= Cfisheye(17) ).*( UV_fisheye(1,:) > 0 ).*...
        ( UV_fisheye(2,:) <= Cfisheye(18) ).*( UV_fisheye(2,:) > 0);
    % Check if point is in front of camera, by checking the sign of the
    % dot-product between the relative position and the forward vector
    %   - vector between point and optical center
    vector_OM = XYZ(1:3,:)-kron(Cfisheye(1:3)',ones(1,size(XYZ,2)));
    %   - unit vector pointing toward z-axis of camera frame
    C2W = CameraToWorld(Cfisheye);
    vector_z_cam = C2W*[0;0;1;1]-Cfisheye(1:3)';
    %   - sign of dot product, converted to provide 0s and 1s
    isInFrontOfCamera = 0.5*(1+sign( dot(vector_OM,kron(vector_z_cam,ones(1,size(XYZ,2)))) ));
    %   - indexes of points that appear in the image
    ind_keep = find(isInsideImage.*isInFrontOfCamera);
    
    %camera calibration
    points = [];
    for i = 2:(grid_size(2)-1)
        for j = 2:(grid_size(1)-1)
            points = [points (j-1)*grid_size(2)+i];
        end
    end; clear i j
    imagePoints(:,1:2,ind_cam) = UV_fisheye(1:2,points)';
    
    
    %% Plot scenario
    % Fisheye example
    clf(gcf);
    set(gcf,'Unit','normalized','Position',[0 0.4 1 0.5])
    subplot(1,2,1); hold all; axis equal; grid on
    xlabel('X','FontSize',14)
    ylabel('Y','Fontsize',14)
    zlabel('Z','Fontsize',14)
    
    % Draw camera
    DrawImagePlane(C);
    
    % Draw World Points
    Points = DrawPoints(XYZ,[0 0 0]);
    
    % Draw image points in world frame
    I2C = ImageToCamera(C);
    C2W = CameraToWorld(C);
    points_world = C2W*I2C*UV(:,ind_keep);
    points_world_fisheye = C2W*I2C*UV_fisheye(:,ind_keep);
    DrawPoints(points_world,[0 1 0]);
    DrawPoints(points_world_fisheye,[1 0 0]);
    
    % Draw corresponding world points
    plot3(XYZ(1,ind_keep),XYZ(2,ind_keep),XYZ(3,ind_keep),...
        'p','MarkerFaceColor','red','MarkerEdgeColor','red','MarkerSize',5);
    Points.XData(ind_keep) = [];Points.YData(ind_keep) = [];Points.ZData(ind_keep) = [];
    
    % Draw Fisheye Lines
    DrawFisheyeProjectionLines([Cfisheye(1),Cfisheye(2),Cfisheye(3)],points_world_fisheye,XYZ(:,ind_keep))
    
    % Limit world
    world_span = max(abs( [XYZ(1,:)-Cfisheye(1),XYZ(2,:)-Cfisheye(2),XYZ(3,:)-Cfisheye(3)]));
    axis([Cfisheye(1)-world_span,Cfisheye(1)+world_span,...
        Cfisheye(2)-world_span,Cfisheye(2)+world_span,...
        Cfisheye(3)-world_span,Cfisheye(3)+world_span])
    view([45,45]);
    
    %% Draw image
    subplot(1,2,2);
    DrawImagePoints(UV(:,ind_keep),Cfisheye(17),Cfisheye(18),[0 1 0]);
    hold on
    DrawImagePoints(UV_fisheye(:,ind_keep),Cfisheye(17),Cfisheye(18),[1 0 0]);
    if ~isempty(UV_fisheye(:,ind_keep))
        legend('Perspective','Fisheye')
    end
    title('Image with Different Camera Model');
    drawnow
    
    %% Save into a .fig file
%     saveas(gcf,['./results/image_calibration_' num2str(ind_cam,'%03d') '.fig']);
%     saveas(gcf,['./results/image_calibration_' num2str(ind_cam,'%03d') '.png']);

    
end

%camera calibration
fisheyeparams = estimateFisheyeParameters(imagePoints,worldPoints,imageSize);
return

%%
%Visualize the calibration accuracy.
figure
showReprojectionErrors(fisheyeparams);

%Visualize camera extrinsics.
figure
showExtrinsics(fisheyeparams);

%Error for fihseye Intrinsics
ErrorMP = Cfisheye(7:10)-fisheyeparams.Intrinsics.MappingCoefficients;
ErrorDC = Cfisheye(11:12)-fisheyeparams.Intrinsics.DistortionCenter;
ErrorSM = [Cfisheye(13) Cfisheye(14);Cfisheye(15) Cfisheye(16)]-fisheyeparams.Intrinsics.StretchMatrix;
ErrorIS = Cfisheye(17:18)-fisheyeparams.Intrinsics.ImageSize;

%Plot detected and reprojected points (First Image)
figure;
hold on;
plot(imagePoints(:,1,1), imagePoints(:,2,1),'go');
plot(fisheyeparams.ReprojectedPoints(:,1,1),fisheyeparams.ReprojectedPoints(:,2,1),'r+');
axis([0 Cfisheye(17) 0 Cfisheye(18)]);
set(gca,'YDir','reverse');
xlabel('u','FontSize',14);
ylabel('v','FontSize',14);
legend('Detected Points','ReprojectedPoints');
title('Detected and Reprojected Points for fisrt image')
hold off;
