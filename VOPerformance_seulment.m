function VOPerformance_seulment(time,Camera,Type)
%This funciton is used to engage single trajectory performance analysis; it
%shows not only trajectory error and TE & RE but also real-time, refined,
%and true trajectory comparision

clearvars -except Camera Type time; close all; clc;

%% Plot Fisheye VO Scenario
filename = ['./results/test_' Camera Type '_' num2str(time,'%03d') '.mat'];
load(filename,'vSet','vSet_RT','groundTruthPoses','t')

%% Real_Time & Refined
[Diff_EstimatedActual_RT,Diff_EstimatedActual_O_RT,TranslationError_RT,RotationError_RT,...
    Distance_RT,DistanceTrue]= Difference_Error(vSet_RT,groundTruthPoses);
[Diff_EstimatedActual_R,Diff_EstimatedActual_O_R,TranslationError_R,RotationError_R,...
    Distance_R,DistanceTrue]= Difference_Error(vSet,groundTruthPoses);

%% Set up Figure (Camera Performance)
figure
% set(gcf,'Position',[10 20 1450 900],'name','Camera Performance')
set(gcf,'outerposition',get(0,'screensize'),'name','Camera Performance')

%% Draw Distance
subplot(3,4,1:2); hold all;
plot(DistanceTrue,'b-','LineWidth',3);
plot(Distance_RT,'y:','LineWidth',3);
plot(Distance_R,'r:','LineWidth',3);
lgnd = legend('True','Real-Time','Refined'); set(lgnd,'color','none');
xlabel('views'); ylabel('Distance (m)'); title('Different Distance')

%% Draw Trajectory
subplot(3,4,3:4); hold all;
location_actual = cat(1,groundTruthPoses.Location{:});
plot3(location_actual(:,1),location_actual(:,2),location_actual(:,3),'b-','LineWidth',3);
locations_RT = cat(1,vSet_RT.Views.Location{:});
plot3(locations_RT(:,1),locations_RT(:,2),locations_RT(:,3),'y:','LineWidth',3);
locations_R = cat(1,vSet.Views.Location{:});
plot3(locations_R(:,1),locations_R(:,2),locations_R(:,3),'r:','LineWidth',3);
view([0,90]); lgnd = legend('True','Real-Time','Refined'); set(lgnd,'color','none');
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)'); title('Camera Trajectory (2D)');

%% Draw Trajectory Error
subplot(3,4,5); hold all;
plot(Diff_EstimatedActual_RT,'.','Color',[0 0.4470 0.7410]);
plot(Diff_EstimatedActual_R,'.','Color',[0 1 1]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('views'); ylabel('error (m)'); title('Estimated Trajectory')
subplot(3,4,6); hold all;
plot(Diff_EstimatedActual_O_RT(:,1),'.','Color',[0.4660 0.6740 0.1880]);
plot(Diff_EstimatedActual_O_R(:,1),'.','Color',[0 1 0]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('views'); ylabel('error (deg)'); title('Estimated Roll')
subplot(3,4,7); hold all;
plot(Diff_EstimatedActual_O_RT(:,2),'.','Color',[0.6350 0.0780 0.1840]);
plot(Diff_EstimatedActual_O_R(:,2),'.','Color',[0.9290 0.6940 0.1250]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('views'); ylabel('error (deg)'); title('Estimated Pitch')
subplot(3,4,8); hold all;
plot(Diff_EstimatedActual_O_RT(:,3),'.','Color'    ,[0.4940 0.1840 0.5560]);
plot(Diff_EstimatedActual_O_R(:,3),'.','Color',[1 0 1]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('views'); ylabel('error (deg)'); title('Estimated Yaw')

%% Draw TE & RE
subplot(3,4,9); hold all
plot(DistanceTrue,TranslationError_RT*100,'.','Color',[0 0.4470 0.7410]);
plot(DistanceTrue,TranslationError_R*100,'.','Color',[0 1 1]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('Path Length [m]'); ylabel('Translation Error [%]'); title('TE vs PL')
subplot(3,4,10); hold all
plot(DistanceTrue,RotationError_RT(:,1),'.','Color',[0.4660 0.6740 0.1880]);
plot(DistanceTrue,RotationError_R(:,1),'.','Color',[0 1 0]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('Path Length [m]'); ylabel('Rotation Error [deg/m]'); title('RE(Roll) vs PL')
subplot(3,4,11); hold all
plot(DistanceTrue,RotationError_RT(:,2),'.','Color',[0.6350 0.0780 0.1840]);
plot(DistanceTrue,RotationError_R(:,2),'.','Color',[0.9290 0.6940 0.1250]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('Path Length [m]'); ylabel('Rotation Error [deg/m]'); title('RE(Pitch) vs PL')
subplot(3,4,12); hold all
plot(DistanceTrue,RotationError_RT(:,3),'.','Color',[0.4940 0.1840 0.5560]);
plot(DistanceTrue,RotationError_R(:,2),'.','Color',[1 0 1]);
lgnd = legend('Real-Time','Refined'); set(lgnd,'color','none');
xlabel('Path Length [m]'); ylabel('Rotation Error [deg/m]'); title('RE(Yaw) vs PL')

%% Time
dim = [.1 .38 .6 .6];
str = ['Running Time: ' num2str(t) ' s'];
annotation('textbox',dim,'string',str,'FitBoxToText','on')

%% Save Figure
saveas(gcf,['./results/' Camera Type '_VOseulment' num2str(time,'%03d') '.png']);

end
