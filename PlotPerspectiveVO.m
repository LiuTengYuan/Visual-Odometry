%% Plot Perspective VO Scenario

clear; close all; clc; tic;

filename = './results/test_perspective.mat';
load(filename)

% h1 = figure;
% h2 = figure;
for ind_pose = 1:Npose
    viewId = ind_pose;
    %% Draw World Scenario
    %     % Perspective example
    %     figure(h1)
    %     clf(gcf);
    %     set(gcf,'Position',[10 20 1450 900])
    %     hold all; grid on;
    %     xlabel('X','FontSize',14)
    %     ylabel('Y','Fontsize',14)
    %     zlabel('Z','Fontsize',14)
    %
    %     % Draw camera
    %     DrawImagePlane(C(ind_pose,:));
    %
    %     % Draw actual camera positions
    %     actual = plot3(x0_v(1:ind_pose),y0_v(1:ind_pose),z0_v(1:ind_pose),'b.-','MarkerSize',12);
    %     % Draw estimated camera positions
    %     locations = cat(1,vSet.Views.Location{:});
    %     estimated = plot3(locations(:,1),locations(:,2),locations(:,3),'g.-','MarkerSize',12);
    %
    %     % Draw World Points
    %     Points = DrawPoints(XYZ,[0 0 0]);
    %
    %     % Draw image points in world frame
    %     I2C = ImageToCamera(C(ind_pose,:));
    %     C2W = CameraToWorld(C(ind_pose,:));
    %     points_world = C2W*I2C*UV{ind_pose}(:,ind_keep{ind_pose});
    %     DrawPoints(points_world,[1 0 0]);
    %
    %     % Draw corresponding world points
    %     plot3(XYZP(1,ind_keep{ind_pose}),XYZP(2,ind_keep{ind_pose}),XYZP(3,ind_keep{ind_pose}),...
    %         'p','MarkerFaceColor','red','MarkerEdgeColor','red','MarkerSize',5);
%     %     Points.XData(ind_keep{ind_pose}) = [];
%     %     Points.YData(ind_keep{ind_pose}) = [];
%     %     Points.ZData(ind_keep{ind_pose}) = [];
    %
    %     % Draw Projection Lines
    %     DrawPerspectiveProjectionLines([C(ind_pose,1),C(ind_pose,2),C(ind_pose,3)],XYZP(:,ind_keep{ind_pose}));
    %
    %     % Limit world
    %     world_span = max(abs( [XYZ(1,:)-C(ind_pose,1),XYZ(2,:)-C(ind_pose,2),XYZ(3,:)-C(ind_pose,3)]));
    %     axis([C(ind_pose,1)-world_span,C(ind_pose,1)+world_span,...
    %         C(ind_pose,2)-world_span,C(ind_pose,2)+world_span,...
    %         C(ind_pose,3)-world_span,C(ind_pose,3)+world_span])
    %
    %     view([45,45]);
    %     legend([actual,estimated]); legend([actual,estimated],{'Actual','Estimated'})
    %     title('Camera Trajectory');
    %     drawnow
    
    %% Draw Camera Trajectory
    %     figure(h2)
    clf(gcf);
    set(gcf,'Position',[10 20 1450 900])
    subplot(2,2,1);
    
    hold all; grid on;
    xlabel('X','FontSize',14)
    ylabel('Y','Fontsize',14)
    zlabel('Z','Fontsize',14)
    actualpos = groundTruthPoses.Location{viewId};
    estimatedpos = vSet.Views.Location{viewId};
    currPosx1 = min(actualpos(1),estimatedpos(1)); currPosx2 = max(actualpos(1),estimatedpos(1));
    currPosy1 = min(actualpos(2),estimatedpos(2)); currPosy2 = max(actualpos(2),estimatedpos(2));
    currPosz1 = min(actualpos(3),estimatedpos(3)); currPosz2 = max(actualpos(3),estimatedpos(3));
    axis([currPosx1-1, currPosx2+1, currPosy1-1, currPosy2+1, currPosz1-1, currPosz2+1]);
    
    cameraSize = 1/10;
    % Plot actual camera pose.
    actualCam = plotCamera('Size', cameraSize, 'Location', ...
        groundTruthPoses.Location{viewId}, 'Orientation', ...
        groundTruthPoses.Orientation{viewId}, 'Color', 'b', 'Opacity', 0);
    % Plot estimated camera pose.
    estimatedCam = plotCamera('Size', cameraSize, 'Location',...
        vSet.Views.Location{viewId}, 'Orientation', vSet.Views.Orientation{viewId},...
        'Color', 'g', 'Opacity', 0);
    
    % Draw actual camera positions
    actual = plot3(x0_v(1:ind_pose),y0_v(1:ind_pose),z0_v(1:ind_pose),'b.-','MarkerSize',12);
    % Draw estimated camera positions
    locations = cat(1,vSet.Views.Location{1:ind_pose});
    estimated = plot3(locations(:,1),locations(:,2),locations(:,3),'g.-','MarkerSize',12);
    view([45,45]);
    legend([actual,estimated]); legend([actual,estimated],{'Actual','Estimated'})
    title('Camera Trajectory');
    
    %% Draw Estimated vs Actual Trajectory
    subplot(2,2,3);
    plot(Diff_EstimatedActual(1:ind_pose),'r-','LineWidth',3);
    xlabel('views'); ylabel('error (m)'); title('Error for Estimated Trajectory')
    
    %% Draw Estimated vs Actual Orientation
    subplot(2,2,4);
    plot(Diff_EstimatedActual_O(1:ind_pose,1),'g-','LineWidth',2); hold on
    plot(Diff_EstimatedActual_O(1:ind_pose,2),'b-','LineWidth',2); hold on
    plot(Diff_EstimatedActual_O(1:ind_pose,3),'y-','LineWidth',2); hold on
    legend('Roll','Pitch','Yaw')
    xlabel('views'); ylabel('error (deg)'); title('Error for Estimated Orientation')
    
    %% Draw image plane
    subplot(2,2,2);
    DrawImagePoints(UV{ind_pose}(:,ind_keep{ind_pose}),C(ind_pose,12),C(ind_pose,13));
    title('Image with Perspective Camera Model');
    drawnow
    
    %% Save into a .fig file
    %     saveas(gcf,['./results/image_perspective_' num2str(ind_pose,'%03d') '.fig']);
    %     saveas(h1,['./results/image1_' num2str(ind_pose,'%03d') '.fig']);
    %     saveas(h2,['./results/image2_' num2str(ind_pose,'%03d') '.fig']);
end; clear ind_pose
subplot(2,2,1);
estimatedCam.Visible = false;
actualCam.Visible = false;
axis tight
% saveas(gcf,['./results/image_perspective_final.fig']);
toc;
return

%% Create animated gif from a list of .fig files
% filename = './results/trajectory_perspective';
% for ind_pose = 1:Npose
%     % open .fig file
%     openfig(['./results/image_perspective_' num2str(ind_pose,'%03d') '.fig']);
%     set(gcf,'renderer','zbuffer')
%     % convert to frame
%     frame = getframe(gcf);
%     im = frame2im(frame);
%     [A,map] = rgb2ind(im,256);
%     % close figure
%     close(gcf);
%     delete(['./results/image_perspective_' num2str(ind_pose,'%03d') '.fig']);
%     % save to movie
%     if ind_pose == 1
%         imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',0);
%     else
%         imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',0);
%     end
% end; clear ind_pose
% return

%% Create .avi movie from a list of .fig files
% filename = './results/trajectory_perspective';
% v = VideoWriter(filename);
% open(v);
% for ind_pose = 1:Npose
%     % open .fig file
%     openfig(['./results/image_perspective_' num2str(ind_pose,'%03d') '.fig']);
%     set(gcf,'renderer','zbuffer')
%     % convert to frame
%     frame = getframe(gcf);
%     % close figure
%     close(gcf);
%     delete(['./results/image_perspective_' num2str(ind_pose,'%03d') '.fig']);
%     % save to movie
%     writeVideo(v,frame);
% end; clear ind_pose
% close(v);
% return
