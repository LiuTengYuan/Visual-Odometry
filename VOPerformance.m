function VOPerformance(Ntimes,Camera,Type)
%This function aims to analyze VO performance from Monte-Carlo
%Simualtion;it shows trajectory error, TE & RE, CDF and PDF

clearvars -except Ntimes Camera Type; close all; clc;

%% Give Parameter of Npose
filenameNpose = ['./results/test_' Camera Type '_' num2str(001,'%03d') '.mat'];
load(filenameNpose,'groundTruthPoses')
Npose = length(groundTruthPoses.ViewId(:));

%% Initialize Translation Error & Rotation Error
TE = zeros(Npose,Ntimes);
RE_Roll = zeros(Npose,Ntimes);
RE_Pitch = zeros(Npose,Ntimes);
RE_Yaw = zeros(Npose,Ntimes);
D = zeros(Npose,Ntimes);
T = zeros(1,Ntimes);

%% Set up Figure for Trajectorty Error
h1 = figure;
set(gcf,'outerposition',get(0,'screensize'),'name',[Camera '_' Type ' camera Trajectory Error'])
subplot(2,3,[1:3]); hold all
xlabel('views'); ylabel('error (m)'); title('Estimated Trajectory');
subplot(2,3,4); hold all
xlabel('views'); ylabel('error (deg)'); title('Estimated Roll')
subplot(2,3,5); hold all
xlabel('views'); ylabel('error (deg)'); title('Estimated Pitch')
subplot(2,3,6); hold all
xlabel('views'); ylabel('error (deg)'); title('Estimated Yaw')

%% Set up Figure for TE % RE
h2 = figure;
set(gcf,'outerposition',get(0,'screensize'),'name',[Camera '_' Type ' camera TE & RE'])
subplot(2,3,[1:3]); hold all
xlabel('Path Length [m]'); ylabel('Translation Error [%]'); title('TE vs PL')
subplot(2,3,4); hold all
xlabel('Path Length [m]'); ylabel('Rotation Error [deg/m]'); title('RE(Roll) vs PL')
subplot(2,3,5); hold all
xlabel('Path Length [m]'); ylabel('Rotation Error [deg/m]'); title('RE(Pitch) vs PL')
subplot(2,3,6); hold all
xlabel('Path Length [m]'); ylabel('Rotation Error [deg/m]'); title('RE(Yaw) vs PL')

%% Plot Trajectory Error, TE RE
for time = 1:Ntimes
    [TE,RE_Roll,RE_Pitch,RE_Yaw,D,T] = PlotError(h1,h2,Camera,Type,time,TE,RE_Roll,RE_Pitch,RE_Yaw,D,T);
end

%% Plot Mean for TE & RE
mean_TE = mean(TE,2);
mean_RE_Raw = mean(RE_Roll,2);
mean_RE_Pitch = mean(RE_Pitch,2);
mean_RE_Yaw = mean(RE_Yaw,2);
mean_D = mean(D,2);
figure(h2);
subplot(2,3,[1:3]); plot(mean_D,mean_TE*100,'o');
subplot(2,3,4); plot(mean_D,mean_RE_Raw,'o');
subplot(2,3,5); plot(mean_D,mean_RE_Pitch,'o');
subplot(2,3,6); plot(mean_D,mean_RE_Yaw,'o');

%% Plot Average Running Time
meanT = mean(T);
dim = [.1 .38 .6 .6];
str = ['Average Running Time: ' num2str(meanT) ' s'];
figure(h1)
annotation('textbox',dim,'string',str,'FitBoxToText','on')
figure(h2)
annotation('textbox',dim,'string',str,'FitBoxToText','on')

%% CDF PDF
h3 = figure;
set(gcf,'units','normalized','outerposition',get(0,'screensize'),'name',[Camera '_' Type ' CDF & PDF'])
subplot(2,4,1); ecdf(TE(Npose,:)); title('CDF Final Translation Error')
subplot(2,4,2); ecdf(RE_Roll(Npose,:)); title('CDF Final Rotation Error (Roll)')
subplot(2,4,3); ecdf(RE_Pitch(Npose,:)); title('CDF Final Rotation Error (Pitch)')
subplot(2,4,4); ecdf(RE_Yaw(Npose,:)); title('CDF Final Rotation Error (Yaw)')
subplot(2,4,5); hist(TE(Npose,:)*100,Ntimes); title('PDF Final Translation Error'); xlabel('TE [%]')
subplot(2,4,6); hist(RE_Roll(Npose,:),Ntimes); title('PDF Final Rotation Error (Roll)'); xlabel('RE(Roll) [deg/m]')
subplot(2,4,7); hist(RE_Pitch(Npose,:),Ntimes); title('PDF Final Rotation Error (Pitch)'); xlabel('RE(Pitch) [deg/m]')
subplot(2,4,8); hist(RE_Yaw(Npose,:),Ntimes); title('PDF Final Rotation Error (Yaw)'); xlabel('RE(Yaw) [deg/m]')

[f_TE,x_TE] = ecdf(TE(Npose,:)); m_TE = mean(TE(Npose,:));
[~,ind67_TE] = min(abs(f_TE-0.67)); error67_TE = x_TE(ind67_TE);
[~,ind95_TE] = min(abs(f_TE-0.95)); error95_TE = x_TE(ind95_TE);

[f_RE1,x_RE1] = ecdf(RE_Roll(Npose,:)); m_RE1 = mean(RE_Roll(Npose,:));
[~,indmean_RE1] = min(abs(x_RE1-m_RE1)); fmean_RE1 = f_RE1(indmean_RE1);
[~,ind67L_RE1] = min(abs(f_RE1-(fmean_RE1+0.335))); error67L_RE1 = x_RE1(ind67L_RE1);
[~,ind67U_RE1] = min(abs(f_RE1-(fmean_RE1-0.335))); error67U_RE1 = x_RE1(ind67U_RE1);
[~,ind95L_RE1] = min(abs(f_RE1-(fmean_RE1+0.475))); error95L_RE1 = x_RE1(ind95L_RE1);
[~,ind95U_RE1] = min(abs(f_RE1-(fmean_RE1-0.475))); error95U_RE1 = x_RE1(ind95U_RE1);

[f_RE2,x_RE2] = ecdf(RE_Pitch(Npose,:)); m_RE2 = mean(RE_Pitch(Npose,:));
[~,indmean_RE2] = min(abs(x_RE2-m_RE2)); fmean_RE2 = f_RE2(indmean_RE2);
[~,ind67L_RE2] = min(abs(f_RE2-(fmean_RE2+0.335))); error67L_RE2 = x_RE2(ind67L_RE2);
[~,ind67U_RE2] = min(abs(f_RE2-(fmean_RE2-0.335))); error67U_RE2 = x_RE2(ind67U_RE2);
[~,ind95L_RE2] = min(abs(f_RE2-(fmean_RE2+0.475))); error95L_RE2 = x_RE2(ind95L_RE2);
[~,ind95U_RE2] = min(abs(f_RE2-(fmean_RE2-0.475))); error95U_RE2 = x_RE2(ind95U_RE2);

[f_RE3,x_RE3] = ecdf(RE_Yaw(Npose,:)); m_RE3 = mean(RE_Yaw(Npose,:));
[~,indmean_RE3] = min(abs(x_RE3-m_RE3)); fmean_RE3 = f_RE3(indmean_RE3);
[~,ind67L_RE3] = min(abs(f_RE3-(fmean_RE3+0.335))); error67L_RE3 = x_RE3(ind67L_RE3);
[~,ind67U_RE3] = min(abs(f_RE3-(fmean_RE3-0.335))); error67U_RE3 = x_RE3(ind67U_RE3);
[~,ind95L_RE3] = min(abs(f_RE3-(fmean_RE3+0.475))); error95L_RE3 = x_RE3(ind95L_RE3);
[~,ind95U_RE3] = min(abs(f_RE3-(fmean_RE3-0.475))); error95U_RE3 = x_RE3(ind95U_RE3);

ErrorTable = table([m_TE;m_RE1;m_RE2;m_RE3],...
    [0 error67_TE;error67L_RE1 error67U_RE1;error67L_RE2 error67U_RE2;error67L_RE3 error67U_RE3],...
    [0 error95_TE;error95L_RE1 error95U_RE1;error95L_RE2 error95U_RE2;error95L_RE3 error95U_RE3],...
    'VariableNames',{'mean','P67','P95'},'Rownames',{'TE','RE_Roll','RE_Pitch','RE_Yaw'})

%% Save
saveas(h1,['./results/' Camera Type '_TrajectoryError.png']);
saveas(h2,['./results/' Camera Type '_TE&RE.png']);
saveas(h3,['./results/' Camera Type '_CDF&PDF.png']);


    function [TE,RE_Raw,RE_Pitch,RE_Yaw,D,T] = PlotError(h1,h2,Camera,Type,time,TE,RE_Raw,RE_Pitch,RE_Yaw,D,T)
        
        filename = ['./results/test_' Camera Type '_' num2str(time,'%03d') '.mat'];
        load(filename,'vSet','vSet_RT','groundTruthPoses','t')
        
        [Diff_EstimatedActual,Diff_EstimatedActual_O,TranslationError,RotationError,Distance,DistanceTrue]...
            = Difference_Error(vSet_RT,groundTruthPoses);
        
        TE(:,time) = TranslationError;
        RE_Raw(:,time) = RotationError(:,1);
        RE_Pitch(:,time) = RotationError(:,2);
        RE_Yaw(:,time) = RotationError(:,3);
        D(:,time) = DistanceTrue;
        T(time) = t;
        
        %% Plot Trajectory Error
        figure(h1);
        subplot(2,3,[1:3]); plot(Diff_EstimatedActual(:),'-','LineWidth',2);
        subplot(2,3,4); plot(Diff_EstimatedActual_O(:,1),'-','LineWidth',2);
        subplot(2,3,5); plot(Diff_EstimatedActual_O(:,2),'-','LineWidth',2);
        subplot(2,3,6); plot(Diff_EstimatedActual_O(:,3),'-','LineWidth',2);
        
        %% Plot Translation & Rotation Error
        figure(h2);
        subplot(2,3,[1:3]); plot(DistanceTrue,TranslationError*100,'.');
        subplot(2,3,4); plot(DistanceTrue,RotationError(:,1),'.');
        subplot(2,3,5); plot(DistanceTrue,RotationError(:,2),'.');
        subplot(2,3,6); plot(DistanceTrue,RotationError(:,3),'.');
        
    end

end
