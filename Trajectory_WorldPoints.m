function Trajectory_WorldPoints(time)

%Random Trajectory & Random World Points (initial condition)

clearvars -except time Ntimes; close all; clc;

%% road simulation in the x-y plane and camera always towards to sky (z)

FrameRate = 30; %(poses/second); 30 or 60
segment = 10; %segments; 10
VS = 10; %(km/hr); range from 5 to 30
sec = 10; %seconds per segment; 10
Npose = segment*sec*FrameRate; %Number of pose totally
curveRadius = 5; %(m) curve radius

% Trajectory: position of x0,y0,z0,a,b,c in world coordinates
[x0_v,y0_v,z0_v,a_v,b_v,c_v,MODE,TrueDistance] = RoadTrajectory(FrameRate,VS,segment,sec,curveRadius,Npose);

%% 3D points in homogeneous coordinate (road scenario)

[XYZ,XYZ_segment,XYZ_L,XYZ_R] = RoadWorldPoints(x0_v,y0_v,z0_v,...
    MODE,TrueDistance,FrameRate,VS,segment,sec,curveRadius,Npose);
Npoints = length(XYZ);
% figure; plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'.'); title('World Points')

% %Random Points
% Npoints = 4000; XYlength = 150; Zlength = 50;
% XYZ(1:2,:) = (rand(2,Npoints)-0.5)*2*XYlength;
% XYZ(3,:) = rand(1,Npoints)*2*Zlength;
% XYZ(4,:) = 1;

%% Create real trajectory data
ViewId = (1:Npose)';
Location = cell(Npose,1);
for i = 1:Npose
    Location(i) = {[x0_v(i),y0_v(i),z0_v(i)]};
end; clear i
Orientation = OrientationWorld2Camera(a_v,b_v,c_v);
%a->roll; b->pitch; c->yaw
groundTruthPoses = table(ViewId,Location,Orientation);

% % Plot of the scenario
% figure;
% for i = 1:Npose;
%     clf(gcf); hold all; grid on
%     xlabel('X','FontSize',14)
%     ylabel('Y','Fontsize',14)
%     zlabel('Z','Fontsize',14)
%     plot3(x0_v(1:i),y0_v(1:i),z0_v(1:i),'b.-','MarkerSize',12);
% %     plot3(x0_v(1:Npose),y0_v(1:Npose),z0_v(1:Npose),'b.-','MarkerSize',12);
% %     plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'k.');
%     cameraSize = 1;
% 
%     plotCamera('Size', cameraSize, 'Location', ...
%         groundTruthPoses.Location{i}, 'Orientation', ...
%         groundTruthPoses.Orientation{i}, 'Color', 'g', 'Opacity', 0);
%     view([45,45]); daspect([1 1 1]);
%     drawnow;
% end; clear i
% return

%% Save data

if exist('time','var')
    filename = ['./results/test_initialcondition_' num2str(time,'%03d') '.mat'];
    save(filename)
else
    filename = './results/test_initialcondition.mat';
    save(filename)
end

end