clear; close all; clc
%% 
% Single landmark
XYZ0 = [10;0;11;1];

% position of the camera
x0 = 10; y0 = 0; z0 = 10;

% attitude of the camera
a0 = 0; b0 = 0; c0 = 0;


%% Perturbation on camera position
% perturbated position of the camera
x1 = x0+0.1*randn(1,1);
y1 = y0+0.1*randn(1,1);
z1 = z0+0.1*randn(1,1);

C = [x0,y0,z0,...
     a0,b0,c0,...
     0.050,...
     1280/2,720/2,...
     1/23e-6,1/23e-6,...
     1280,720];

C2 = [x1,y1,z1,...
      a0,b0,c0,...
      0.050,...
      1280/2,720/2,...
      1/23e-6,1/23e-6,...
      1280,720];

% Project points onto the image plane for first cam
P = PerspectiveProjectionMatrix(C);
UV = P*XYZ0;
for ind_points = 1:size(UV,2);
    UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
end; clear ind_points

% Project points onto the image plane for first cam
P2 = PerspectiveProjectionMatrix(C2);
UV2 = P2*XYZ0;
for ind_points = 1:size(UV2,2);
    UV2(:,ind_points) = UV2(:,ind_points)/UV2(3,ind_points);
end; clear ind_points

% computation of Jacobian
[J_pos,J_att,J_lmk] = PerspectiveProjectionJacobian(C,XYZ0);
% computation of the perturbation in the measurement domain
delta_UV = J_pos*[x1-x0;y1-y0;z1-z0];

figure; hold all
DrawImagePoints(UV,C(12),C(13));
DrawImagePoints(UV2,C(12),C(13));
quiver(UV(1),UV(2),delta_UV(1),delta_UV(2));

%% Perturbation on camera attitude
% perturbated position of the camera
a1 = a0+0.1*randn(1,1);
b1 = b0+0.1*randn(1,1);
c1 = c0+0.1*randn(1,1);

C = [x0,y0,z0,...
   a0,b0,c0,...
   0.050,...
   1280/2,720/2,...
   1/23e-6,1/23e-6,...
   1280,720];

C2 = [x0,y0,z0,...
   a1,b1,c1,...
   0.050,...
   1280/2,720/2,...
   1/23e-6,1/23e-6,...
   1280,720];

% Project points onto the image plane for first cam
P = PerspectiveProjectionMatrix(C);
UV = P*XYZ0;
for ind_points = 1:size(UV,2);
    UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
end; clear ind_points

% Project points onto the image plane for 2nd cam
P2 = PerspectiveProjectionMatrix(C2);
UV2 = P2*XYZ0;
for ind_points = 1:size(UV2,2);
    UV2(:,ind_points) = UV2(:,ind_points)/UV2(3,ind_points);
end; clear ind_points

% computation of Jacobian
[J_pos,J_att,J_lmk] = PerspectiveProjectionJacobian(C,XYZ0);
% computation of the perturbation in the measurement domain
delta_UV = J_att*[a1-a0;b1-b0;c1-c0];

figure; hold all
DrawImagePoints(UV,C(12),C(13));
DrawImagePoints(UV2,C(12),C(13));
quiver(UV(1),UV(2),delta_UV(1),delta_UV(2));


%% Perturbation on landmark position
% perturbated landmark
XYZ1 = XYZ0 + [0.1*randn(3,1);0];

C=[x0,y0,z0,...
   a0,b0,c0,...
   0.050,...
   1280/2,720/2,...
   1/23e-6,1/23e-6,...
   1280,720];

% Project points onto the image plane for first cam
P = PerspectiveProjectionMatrix(C);
UV = P*XYZ0;
for ind_points = 1:size(UV,2);
    UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
end; clear ind_points

% Project points onto the image plane for 2nd cam
UV2 = P*XYZ1;
for ind_points = 1:size(UV2,2);
    UV2(:,ind_points) = UV2(:,ind_points)/UV2(3,ind_points);
end; clear ind_points

% computation of Jacobian
[J_pos,J_att,J_lmk] = PerspectiveProjectionJacobian(C,XYZ0);
% computation of the perturbation in the measurement domain
delta_UV = J_lmk*(XYZ1(1:3)-XYZ0(1:3));

figure; hold all
DrawImagePoints(UV,C(12),C(13));
DrawImagePoints(UV2,C(12),C(13));
quiver(UV(1),UV(2),delta_UV(1),delta_UV(2));

