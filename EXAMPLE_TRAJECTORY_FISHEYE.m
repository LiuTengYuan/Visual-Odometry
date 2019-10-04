% EXAMPLE of the computation projection of points for the perspective camera models

clear all
close all
clc

filename = './results/trajectory.gif';

%%
FrameRate = 30; %(poses/second); 30 or 60
VS = 10; %(km/hr); range from 5 to 30
segment = 10; %segments; 10
sec = 10; %seconds per segment; 10
Npose = segment*sec*FrameRate; %Number of pose totally
curveRadius = 5; %(m) curve radius
% Trajectory: position of x0,y0,z0,a,b,c in world coordinates
[x0_v,y0_v,z0_v,a_v,b_v,c_v,MODE,TrueDistance] = RoadTrajectory(FrameRate,VS,segment,sec,curveRadius,Npose);
[XYZ,XYZ_segment] = RoadWorldPoints(x0_v,y0_v,z0_v,MODE,TrueDistance,FrameRate,VS,segment,sec,curveRadius,Npose);
Npoints = length(XYZ);

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
    % Fisheye Camera Parameters: C=[x0,y0,z0,a,b,c,a0,a2,a3,a4,cx,cy,s1,s2,s3,s4,Nu,Nv];
    % x0,y0,z0   : (1-3)   location (in meters)
    % a,b,c      : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % a0,a2,a3,a4: (7-10)  mapping coefficeients: Scaramuzza's Taylor model for projection function
    % cx,cy      : (11-12) center of distortion (in pixels)
    % s1,s2,s3,s4: (13-16) stretch matrix: 2-by-2 transformation matrix from sensor plane to camera image plane
    % Nu,Nv      : (17-18) Number of pixels
    
    Cfisheye=[x0_v(ind_pose),y0_v(ind_pose),z0_v(ind_pose),...
        a_v(ind_pose),b_v(ind_pose),c_v(ind_pose),...
        875.08,-3.04e-3,-4.82e-08,1.71e-11,...%a0,a2,a3,a4     500.08,-1.02e-3,-3.82e-06,1.71e-10,...
        640,360,...% cx,cy
        1,0,0,1,...% s1,s2,s3,s4
        1280,720];% Nu,Nv
    
    %% Project points onto the image plane
    
    % Perspective Camera
    P = PerspectiveProjectionMatrix(C);
    XYZ_try = [XYZ_segment{1},XYZ_segment{2},XYZ_segment{3}];
    UV = P*XYZ_try;
    for ind_points = 1:size(UV,2);
        UV(:,ind_points) = UV(:,ind_points)/UV(3,ind_points);
    end; clear ind_points
    
    % Fisheye Camera
    UV_fisheye = FisheyeDistortion(XYZ_try,Cfisheye,C);
    
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
    ind_keep = find(isInsideImage);
    
    
    %% Plot scenario
    % Fisheye example
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
    Points = DrawPoints(XYZ_try,[0 0 0]);
    
    % Draw image points in world frame
    I2C = ImageToCamera(C);
    C2W = CameraToWorld(C);
    points_world = C2W*I2C*UV(:,ind_keep);
    points_world_fisheye = C2W*I2C*UV_fisheye(:,ind_keep);
    DrawPoints(points_world,[0 1 0]);
    DrawPoints(points_world_fisheye,[1 0 0]);
    
    % Draw corresponding world points
    plot3(XYZ_try(1,ind_keep),XYZ_try(2,ind_keep),XYZ_try(3,ind_keep),...
        'p','MarkerFaceColor','red','MarkerEdgeColor','red','MarkerSize',5);
    Points.XData(ind_keep) = [];Points.YData(ind_keep) = [];Points.ZData(ind_keep) = [];
    
    % Draw Fisheye Lines
    DrawFisheyeProjectionLines([Cfisheye(1),Cfisheye(2),Cfisheye(3)],points_world_fisheye,XYZ_try(:,ind_keep))
    
    % Limit world
    world_span = max(abs( [XYZ_try(1,:)-Cfisheye(1),XYZ_try(2,:)-Cfisheye(2),...
        XYZ_try(3,:)-Cfisheye(3)]));
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


