% EXAMPLE of the computation projection of points for the perspective camera models

clear all
close all
clc

filename = './results/trajectory.gif';

%% Scenario definition

% Trajectory: position of x0,y0,z0,a,b,c in world coordinates
% %   - straight line along x_world
% Npose = 100;
% x0_v = -Npose*0.1+(0:Npose-1)*0.1;
% y0_v = zeros(1,Npose);
% z0_v = 2*ones(1,Npose);
% a_v = pi/2*ones(1,Npose);
% b_v = 0*pi/2*ones(1,Npose);
% c_v = pi/2*ones(1,Npose);

% %   - circle in the x-y plane
% Npose = 100;
% x0_v = cos(2*pi*(0:Npose-1)/Npose);
% y0_v = sin(2*pi*(0:Npose-1)/Npose);
% z0_v = 2*ones(1,Npose);
% a_v = -pi/2*ones(1,Npose);
% b_v = zeros(1,Npose);2*pi*(0:Npose-1)/Npose;
% c_v = 2*pi*(0:Npose-1)/Npose;

%   - 8-shape in the x-y plane
Npose = 100;
x0_v = cos(2*pi*(0:Npose-1)/Npose);
y0_v = sin(2*2*pi*(0:Npose-1)/Npose);
z0_v = 2*ones(1,Npose);
a_v = -pi/2*ones(1,Npose);
b_v = zeros(1,Npose);
c_v = atan2(sin(2*pi*(0:Npose-1)/Npose),cos(2*2*pi*(0:Npose-1)/Npose));

% %   - stationary circle in the x-y plane
% Npose = 100;
% x0_v = zeros(1,Npose);
% y0_v = zeros(1,Npose);
% z0_v = 2*ones(1,Npose);
% a_v = 1*pi/2*ones(1,Npose);
% b_v = 2*pi*(0:Npose-1)/Npose;
% c_v = zeros(1,Npose);

% % set initial position and orientation to suit normalization
% increment = 50;
% Npose = Npose+increment;
% dx = (x0_v(1)-0)/increment; x0 = x0_v(1);
% dy = (y0_v(1)-0)/increment; y0 = y0_v(1);
% dz = (z0_v(1)-0)/increment; z0 = z0_v(1);
% da_v = (a_v(1)-0)/increment; a0 = a_v(1);
% db_v = (b_v(1)-0)/increment; b0 = b_v(1);
% dc_v = (c_v(1)-0)/increment; c0 = c_v(1);
% for i = 1:increment
%     x0_v = [x0-dx*i x0_v];
%     y0_v = [y0-dy*i y0_v];
%     z0_v = [z0-dz*i z0_v];
%     a_v = [a0-da_v*i a_v];
%     b_v = [b0-db_v*i b_v];
%     c_v = [c0-dc_v*i c_v];
% end; clear i

% 3D points in homogeneous coordinate: randomly placed points
Npoints = 200;
XYZ(1:2,:) = (rand(2,Npoints)-0.5)*10;
XYZ(3,:) = 2 + (rand(1,Npoints)-0.5)*4;
XYZ(4,:) = 1;

% % Plot of the scenario
% figure; hold all; axis equal; grid on
% plot3(x0_v,y0_v,z0_v);
% plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'.');
% view([45 45])
% xlabel('X','FontSize',14)
% ylabel('Y','Fontsize',14)
% zlabel('Z','Fontsize',14)
% return
%%
FRAMES(Npose) = struct('cdata',[],'colormap',[]);
for ind_pose = 1:Npose
    %% Camera Parameters: C=[x0,y0,z0,a,b,g,f,u0,v0,ku,kv];
    % x0,y0,z0: (1-3)   location (in meters)
    % a,b,c   : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % f       : (7)     focal length (in m)
    % u0,v0   : (8-9)   optical center (in pixels)
    % ku,kv   : (10-11) inverse of pixel size (in pixels/m)
    % Nu,Nv   : (12-13) Number of pixels
    
    C=[x0_v(ind_pose),y0_v(ind_pose),z0_v(ind_pose),...
        a_v(ind_pose),b_v(ind_pose),c_v(ind_pose),...
        0.05,... %0.025,...
        1280/2,720/2,...
        1/23e-6,1/23e-6,...
        1280,720];
    
    %% Project points onto the image plane
    % Perspective Projection matrix
    P = PerspectiveProjectionMatrix(C);
    
    % Project into image frame, in pixels
    UV = P*XYZ;
    
    % Normalize by the 3rd homogeneous coordintate
    for ind_points = 1:size(UV,2)
        UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
    end; clear ind_points
    
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
    isInFrontOfCamera = 0.5*(1+sign( dot(vector_OM,kron(vector_z_cam,ones(1,size(XYZ,2))) ) ));
    %   - indexes of points that appear in the image
    ind_keep = find(isInsideImage.*isInFrontOfCamera);
    
    
    %% Plot scenario
    % Perspective example
    clf(gcf);
    set(gcf,'Unit','normalized','Position',[0 0.4 1 0.5])
    subplot(1,2,1);
    hold all; axis equal; grid on;
    xlabel('X','FontSize',14)
    ylabel('Y','Fontsize',14)
    zlabel('Z','Fontsize',14)
    
    % Draw camera
    DrawImagePlane(C);
    
    % Draw previous camera positions
    plot3(x0_v(1:ind_pose),y0_v(1:ind_pose),z0_v(1:ind_pose),'b.-');
    
    % Draw World Points
    Points = DrawPoints(XYZ,[0 0 0]);
    
    % Draw image points in world frame
    I2C = ImageToCamera(C);
    C2W = CameraToWorld(C);
    points_world = C2W*I2C*UV(:,ind_keep);
    DrawPoints(points_world,[1 0 0]);
    
    % Draw corresponding world points
    plot3(XYZ(1,ind_keep),XYZ(2,ind_keep),XYZ(3,ind_keep),...
        'p','MarkerFaceColor','red','MarkerEdgeColor','red','MarkerSize',5);
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
    subplot(1,2,2);
    DrawImagePoints(UV(:,ind_keep),C(12),C(13));
    drawnow
    
    %% Save into a .fig file
    %     savefig(['./results/image_' num2str(ind_pose,'%03d') '.fig']);
    
end; clear ind_pose
return

%% Create animated gif from a list of .fig files
% for ind_pose = 1:Npose
%     % open .fig file
%     openfig(['./results/image_' num2str(ind_pose,'%03d') '.fig']);
%     set(gcf,'renderer','zbuffer')
%     % convert to frame
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [A,map] = rgb2ind(im,256);
%     % close figure
%     close(gcf);
%     % save to movie
%     if ind_pose == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0);
%     end
% end; clear ind_pose
% return

%% Create .avi movie from a list of .fig files
% v = VideoWriter(filename);
% open(v);
% for ind_pose = 1:Npose
%     % open .fig file
%     openfig(['./results/image_' num2str(ind_pose,'%03d') '.fig']);
%     set(gcf,'renderer','zbuffer')
%     % convert to frame
%     frame = getframe(gcf);
%     % close figure
%     close(gcf);
%     % save to movie
%     writeVideo(v,frame);
% end; clear ind_pose
% close(v);
% return


