%Monte-Carlo Simulation

clear; close all; clc; 
% tic;

%% Visual Odometry (Perspective & Fisheye)
% profile on;
Ntimes = 1; %number of running times
for time = 1:Ntimes
    
    %calling Trajectory_WorldPoints to create initial condition(trajectory & world points)
    Trajectory_WorldPoints(time);
%     %calling Perspective Camera VO
%     VOPerspective(time);
    %calling Fisheye Camera VO
    % second parameter is ratio of velcoity(oringinal V->10km/hr)
    % third parameter is window size(bundlement adjustment)
    % fourth parameter is to check if we use normalization (groundTruth)
    VOFisheye(time,1,15,'N');
    VOFisheye(time,1,0,'N');
    VOFisheye(time,1,30,'N');
    VOFisheye(time,2,15,'N');
    VOFisheye(time,2,30,'N');
    VOFisheye(time,1,15,'');
end
% profile off; profsave;
% toc;
return
%% VO Performance

% % Perspective Camera
% VOPerformance(Ntimes,'perspective');
% Fisheye Camera
VOPerformance(Ntimes,'fisheye','VR1WZ15N');
VOPerformance(Ntimes,'fisheye','VR1WZ0N');
VOPerformance(Ntimes,'fisheye','VR1WZ30N');
VOPerformance(Ntimes,'fisheye','VR2WZ15N');
VOPerformance(Ntimes,'fisheye','VR2WZ30N');
VOPerformance(Ntimes,'fisheye','VR1WZ15');

% VOPerformance_seulment(time,'fisheye','VR1WZ15N');
% VOPerformance_seulment(time,'fisheye','VR1WZ0N');
% VOPerformance_seulment(time,'fisheye','VR1WZ30N');
% VOPerformance_seulment(time,'fisheye','VR2WZ15N');
% VOPerformance_seulment(time,'fisheye','VR2WZ30N');
% VOPerformance_seulment(time,'fisheye','VR1WZ15');
