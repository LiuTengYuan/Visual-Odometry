% EXAMPLE of the computation projection of points for the perspective camera models
% open cameraIntrinsics
clear all
close all
clc

%% Scenario definition

% Camera Parameters: C=[x0,y0,z0,a,b,c,f,u0,v0,ku,kv];
% x0,y0,z0: (1-3)   location (in meters)
% a,b,c   : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
% f       : (7)     focal length (in m)
% u0,v0   : (8-9)   optical center (in pixels)
% ku,kv   : (10-11) inverse of pixel size (in pixels/m)
% Nu,Nv   : (12-13) Number of pixels

C=[0,0,0,...%x0,y0,z0
    0*pi/180,0*pi/180,0*pi/180,...%a,b,c
    0.050,...% f
    1280/2,720/2,...% u0,v0
    1/23e-6,1/23e-6,...% ku,kv
    1280,720];% Nu,Nv

% Radio Distortion: Rd = [k1,k2,k3]
% x_distorted = x*(1+k1*r^2+k2*r^4+k3*r^6)
% y_distorted = y*(1+k1*r^2+k2*r^4+k3*r^6)
% r^2 = x^2+y^2
% (x,y): undistored pixel locations, x and y are in normalized image
% coordinates, which are cacualted from pixel coordinates by translating to
% the optical center and divided by the focal length in pixels. Thus x and
% y are dimensionless
% (k1,k2,k3): radio distortion coefficients

Rd = [3, 0.5, 0.1];

% Tangential Distortion: Td = [p1 p2]
% x_distorted = x+[2*p1*x*y+p2*(r^2+2*x^2)]
% y_distorted = y+[p1*(r^2+2*y^2)+2*p2*x*y]
% r^2 = x^2+y^2
% (x,y): undistored pixel locations, x and y are in normalized image
% coordinates, which are cacualted from pixel coordinates by translating to
% the optical center and divided by the focal length in pixels. Thus x and
% y are dimensionless
% (p1,p2): tangential distortion coefficients

Td = [0, 0];

% 3D points in homogeneous coordinate: planar grid
grid_size = [2,10]; % size of grid
grid_spacing = 0.01; % spacing between grid points
grid_origin = [-0.005 -0.05];
XYZ(1,:) = grid_origin(1) + kron((1:grid_size(1))*grid_spacing, ones(1,grid_size(2)));
XYZ(2,:) = grid_origin(2) + repmat((1:grid_size(2))*grid_spacing, 1, grid_size(1));
XYZ(3,:) = 0.1;
XYZ(4,:) = 1;

% % Randomly distributed points
% Npoints = 1000;
% XYZ(1:2,:) = (rand(2,Npoints)-0.5)*0.2;
% XYZ(3,:) = (rand(1,Npoints)-0.5)*0.2;
% XYZ(4,:) = 1;

%%
% FrameRate = 30; %(poses/second); 30 or 60
% VS = 10; %(km/hr); range from 5 to 30
% segment = 10; %segments; 10
% sec = 10; %seconds per segment; 10
% Npose = segment*sec*FrameRate; %Number of pose totally
% curveRadius = 5; %(m) curve radius
% % Trajectory: position of x0,y0,z0,a,b,c in world coordinates
% [x0_v,y0_v,z0_v,a_v,b_v,c_v,MODE] = RoadTrajectory(FrameRate,VS,segment,sec,curveRadius,Npose);
% [XYZ,XYZ_segment] = RoadWorldPoints(x0_v,y0_v,z0_v,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
% Npoints = length(XYZ);
%% Project points onto the image plane
% Perspective Projection matrix
P = PerspectiveProjectionMatrix(C);

UV = P*XYZ;
for ind_points = 1:size(UV,2);
    UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
end; clear ind_points
% Undistortion Image Coordinates
UUVV = UV;

%Radial & Tangential Distortion
UV = RadialTangentialDistortion(C,UV,Rd,Td);

% Exclude points outside the image
isInsideImage = ( UV(1,:) <= C(12) ).*( UV(1,:) > 0 ).*( UV(2,:) <= C(13) ).*( UV(2,:) > 0);
% Check if point is in front of camera, by checking the sign of the
% dot-product between the relative position and the forward vector
%   - vector between point and optical center
vector_OM = XYZ(1:3,:)-kron(C(1:3)',ones(1,size(XYZ,2)));
%   - unit vector pointing toward z-axis of camera frame
C2W = CameraToWorld(C);
vector_z_cam = C2W*[0;0;1;1]-C(1:3)';
%   - sign of dot product, converted to provide 0s and 1s
isInFrontOfCamera = 0.5*(1+sign( dot(vector_OM,kron(vector_z_cam,ones(1,size(XYZ,2)))) ));
%   - indexes of points that appear in the image
ind_keep = find(isInsideImage.*isInFrontOfCamera);


%% Plot scenario
% Perspective example
figure(1); hold all; axis equal; grid on
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
points_world_undistored = C2W*I2C*UUVV(:,ind_keep);
DrawPoints(points_world,[1 0 0]);
DrawPoints(points_world_undistored,[0 1 0]);

% Draw corresponding world points
plot3(XYZ(1,ind_keep),XYZ(2,ind_keep),XYZ(3,ind_keep),...
    'p','MarkerFaceColor','green','MarkerEdgeColor','green','MarkerSize',5);
Points.XData(ind_keep) = [];Points.YData(ind_keep) = [];Points.ZData(ind_keep) = [];

% Draw Projection Lines
DrawPerspectiveProjectionLines([C(1),C(2),C(3)],XYZ(:,ind_keep));

% Limit world
world_span = max(abs( [XYZ(1,:)-C(1),XYZ(2,:)-C(2),XYZ(3,:)-C(3)]));
axis([C(1)-world_span,C(1)+world_span,...
    C(2)-world_span,C(2)+world_span,...
    C(3)-world_span,C(3)+world_span])
view([45,45]);

%% Draw image
figure;
DrawImagePoints(UV(:,ind_keep),C(12),C(13),[1 0 0]); %distored(red)
hold on 
DrawImagePoints(UUVV(:,ind_keep),C(12),C(13),[0 1 0]); %undistored(green)
if ~isempty(UUVV(:,ind_keep))
    legend('Distored','Undistored')
end
title('Image with Perspective Camera Model');
