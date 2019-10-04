function VOFisheye_2D2D(time)
%VO by 2D-2D motion estimation; however ,there are still some unsolved
%error when we apply this funciton


% dbstop if error

%Output #5: VO script using the fisheye camera simulator + example of VO output on a simple trajectory
dbstop if error
clearvars -except time Ntimes; close all; clc;

if exist('time','var')
    filename = ['./results/test_initialcondition_' num2str(time,'%03d') '.mat'];
    load(filename)
else
    filename = './results/test_initialcondition.mat';
    load(filename)
end

%% Create the perspective & fisheye camera parameters object using camera intrinsics
f=0.05;                    %focal length (in m)
u0=1280/2; v0=720/2;       %optical center (in pixels)
ku=1/23e-6; kv=1/23e-6;    %inverse of pixel size (in pixels/m)
Nu=1280; Nv=720;           %Number of pixels
Rd = [0 0 0];              %Radio Distortion Coefficients
Td = [0 0];                %Tangential Distortion Coefficients
K = [f*ku 0 u0; 0 f*kv v0; 0 0 1]'; %Intrinsic Matrix
cameraParams = cameraParameters('IntrinsicMatrix', K, 'RadialDistortion', Rd, 'TangentialDistortion', Td,...
    'ImageSize', [Nv Nu], 'NumRadialDistortionCoefficients', 3, 'EstimateTangentialDistortion', true);

a0 = 875.08; a2 = -3.04e-3; a3 = -4.82e-08; a4 = 1.71e-11; %Mapping Coefficients
cx = 640; cy = 360;                                        %Center Distortion(in pixels)
s1 = 1; s2 = 0; s3 = 0; s4 = 1;                            %Stretch Matrix
SM = [s1 s2;s3 s4];
intrinsics = fisheyeIntrinsics([a0, a2, a3, a4], [Nv, Nu], [cx, cy], SM);
fisheyeParams = fisheyeParameters(intrinsics);

%%
vSet = viewSet;
ind_keep = cell(Npose,1);
C = zeros(Npose,13);
Cfisheye = zeros(Npose,18);
XYZfisheye = cell(Npose,1);
XYZfisheyeP = cell(Npose,1);
UV = cell(Npose,1);
UV_fisheye = cell(Npose,1);

Diff_EstimatedActual = NaN(height(groundTruthPoses),1); %(m)
Diff_EstimatedActual_O = NaN(height(groundTruthPoses),3); %(deg)
Distance = NaN(height(groundTruthPoses),1); %(m)
TranslationError = NaN(height(groundTruthPoses),1); %(%)
RotationError = NaN(height(groundTruthPoses),3); %(deg/m) 

for ind_pose = 1:Npose
    %% Perspective Camera Parameters: C=[x0,y0,z0,a,b,c,f,u0,v0,ku,kv,Nu,Nv];
    % x0,y0,z0: (1-3)   location (in meters)
    % a,b,c   : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % f       : (7)     focal length (in m)
    % u0,v0   : (8-9)   optical center (in pixels)
    % ku,kv   : (10-11) inverse of pixel size (in pixels/m)
    % Nu,Nv   : (12-13) Number of pixels
    
    C(ind_pose,:)=[x0_v(ind_pose),y0_v(ind_pose),z0_v(ind_pose),...
        a_v(ind_pose),b_v(ind_pose),c_v(ind_pose),...
        f,...
        u0,v0,...
        ku,kv,...
        Nu,Nv];
    %% Fisheye Camera Parameters: Cfisheye=[x0,y0,z0,a,b,c,a0,a2,a3,a4,cx,cy,s1,s2,s3,s4,Nu,Nv];
    % Fisheye Camera Parameters: C=[x0,y0,z0,a,b,c,a0,a2,a3,a4,cx,cy,s1,s2,s3,s4,Nu,Nv];
    % x0,y0,z0   : (1-3)   location (in meters)
    % a,b,c      : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % a0,a2,a3,a4: (7-10)  mapping coefficeients: Scaramuzza's Taylor model for projection function
    % cx,cy      : (11-12) center of distortion (in pixels)
    % s1,s2,s3,s4: (13-16) stretch matrix: 2-by-2 transformation matrix from sensor plane to camera image plane
    % Nu,Nv      : (17-18) Number of pixels
    
    Cfisheye(ind_pose,:)=[x0_v(ind_pose),y0_v(ind_pose),z0_v(ind_pose),...
        a_v(ind_pose),b_v(ind_pose),c_v(ind_pose),...
        a0,a2,a3,a4,...
        cx,cy,...
        s1,s2,s3,s4,...
        Nu,Nv];
    
    %% Project points onto the image plane
    % Perspective Projection matrix
    P = PerspectiveProjectionMatrix(C(ind_pose,:));
    
    %Road Scenario(choose world points from segments)
    Segment = ceil(ind_pose/(Npose/segment));
    if Segment > 1 && Segment < segment
        XYZfisheye{ind_pose} = [XYZ_segment{Segment-1},XYZ_segment{Segment},XYZ_segment{Segment+1}];
    elseif Segment == 1
        XYZfisheye{ind_pose} = [XYZ_segment{Segment},XYZ_segment{Segment+1}];
    else %Segment = segment
        XYZfisheye{ind_pose} = [XYZ_segment{Segment-1},XYZ_segment{Segment}];
    end
    
    %Road Scenario(block world points penertating buildings)
    currPos = [x0_v(ind_pose) y0_v(ind_pose) z0_v(ind_pose)];
    XYZfisheyeP{ind_pose} = CheckPenetratePoints(currPos, XYZfisheye{ind_pose},...
        XYZ_L{Segment}, XYZ_R{Segment});
    
    % Project into image frame, in pixels
    UV(ind_pose) = {P*XYZfisheyeP{ind_pose}};
    % Normalize by the 3rd homogeneous coordintate
    for ind_points = 1:size(UV{ind_pose},2)
        UV{ind_pose}(:,ind_points) = UV{ind_pose}(:,ind_points)/UV{ind_pose}(3,ind_points);
    end; clear ind_points
    
    % Fisheye Camera
    UV_fisheye(ind_pose) = {FisheyeDistortion(XYZfisheyeP{ind_pose},Cfisheye(ind_pose,:),C(ind_pose,:))};
    
    % Exclude points outside the image
    isInsideImage = ( UV_fisheye{ind_pose}(1,:) <= Cfisheye(ind_pose,17) ).*( UV_fisheye{ind_pose}(1,:) > 0 ).*...
        ( UV_fisheye{ind_pose}(2,:) <= Cfisheye(ind_pose,18) ).*( UV_fisheye{ind_pose}(2,:) > 0);
    %   - indexes of points that appear in the image
    ind_keep{ind_pose} = find(isInsideImage);
    
    %% Visual Odometry
    
    viewId = ind_pose;
    
    PerspectivePoints = UV{ind_pose}(1:2,ind_keep{ind_pose})';
    fisheyePoints = UV_fisheye{ind_pose}(1:2,ind_keep{ind_pose})';
    
%     %select uniform fisheye points!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
%     %there is one problem that it makes world points from double to single
%     %????????????????????????????????????????????????????????????????????
%     numPoints = height(groundTruthPoses);
%     SURFfisheyePoints = SURFPoints(fisheyePoints);
%     uniformfisheyePoints = selectUniform(SURFfisheyePoints,numPoints,[Cfisheye(ind_pose,18) Cfisheye(ind_pose,17)]);
%     fisheyecurrPoints = uniformfisheyePoints.Location;
%     indexPairsUniform = DetectandMatchFeatures(fisheyecurrPoints',single(fisheyePoints)');
%     %???????????????????????????????????????????????????????????????????? 
%     %plot before uniform & after uniform points (testing)
%     figure; hold all; 
%     plot(fisheyePoints(:,1),fisheyePoints(:,2),'.')
%     plot(fisheyecurrPoints(:,1),fisheyecurrPoints(:,2),'.')
%     legend('Before Unifrom','After Uniform'); title('SelectUniform'); 
    
    %??????????????????????????????????????????????????????????????????????
    scaleFactor = [C(ind_pose,7)*C(ind_pose,10) C(ind_pose,7)*C(ind_pose,11)]...
        /(min([C(ind_pose,13) C(ind_pose,12)])/2);
    scaleFactor = 1;
    %??????????????????????????????????????????????????????????????????????
    
    [currPoints, camIntrinsics, reprojectionErrors] = ...
        undistortFisheyePoints(fisheyePoints, fisheyeParams.Intrinsics, scaleFactor);
    currFeatures = XYZfisheyeP{ind_pose}(1:3,ind_keep{ind_pose});
%     currFeatures = XYZfisheyeP{ind_pose}(1:3,ind_keep{ind_pose}(indexPairsUniform(:,2)));
    
    %     %plot image points (testing)
    %     figure; set(gcf,'Position',[10 20 1450 900])
    %     subplot(2,2,1); plot(PerspectivePoints(:,1),PerspectivePoints(:,2),'.')
    %     title('Perspective Points'); daspect([1 1 1]);
    %     subplot(2,2,[2]); plot(fisheyecurrPoints(:,1),fisheyecurrPoints(:,2),'.')
    %     title('Distorted Fisheye Points'); daspect([1 1 1]);
    %     subplot(2,2,3); plot(currPoints(:,1),currPoints(:,2),'.')
    %     title('Undistorted Fisheye Points'); daspect([1 1 1]);
    %     subplot(2,2,4); plot3(currFeatures(1,:),currFeatures(2,:),currFeatures(3,:),'.')
    %     title('World Points'); daspect([1 1 1]);
    
    switch ind_pose
        case 1
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', ...
                groundTruthPoses.Orientation{1}, 'Location', groundTruthPoses.Location{1});
        case 2
            indexPairs = DetectandMatchFeatures(preFeatures,currFeatures);
            [orient, loc, inlierIdx] = helperEstimateRelativePose(...
                prevPoints(indexPairs(:,1),:), currPoints(indexPairs(:,2),:), cameraParams);
            loc = loc+vSet.Views.Location{viewId-1};
            orient = orient*vSet.Views.Orientation{viewId-1};
            indexPairs = indexPairs(inlierIdx, :);
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
                'Location', loc);
            vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
            vSet = helperNormalizeViewSet(vSet, groundTruthPoses);
        case num2cell(3:15)
            indexPairs = DetectandMatchFeatures(preFeatures,currFeatures);
            [orient, loc, inlierIdx] = helperEstimateRelativePose(...
                prevPoints(indexPairs(:,1),:), currPoints(indexPairs(:,2),:), cameraParams);
            loc = loc+vSet.Views.Location{viewId-1};
            orient = orient*vSet.Views.Orientation{viewId-1};
            indexPairs = indexPairs(inlierIdx, :);
%             [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences_liu(vSet,...
%                 cameraParams, indexPairs, currPoints);
%             warningstate = warning('off','vision:ransac:maxTrialsReached');
%             [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
%                 cameraParams, 'Confidence', 99.99, 'MaxReprojectionError', 0.8);
%             warning(warningstate)
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
                'Location', loc);
            vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
            tracks = findTracks(vSet);
            camPoses = poses(vSet);
            xyzPoints = triangulateMultiview(tracks, camPoses, cameraParams);
            [~, camPoses] = bundleAdjustment(xyzPoints, tracks, camPoses, ...
                cameraParams, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
                'RelativeTolerance', 1e-9, 'MaxIterations', 300);
            vSet = updateView(vSet, camPoses);
            vSet = helperNormalizeViewSet(vSet, groundTruthPoses);
        case num2cell(16:Npose)
            indexPairs = DetectandMatchFeatures(preFeatures,currFeatures);
            [orient, loc, inlierIdx] = helperEstimateRelativePose(...
                prevPoints(indexPairs(:,1),:), currPoints(indexPairs(:,2),:), cameraParams);
            loc = loc+vSet.Views.Location{viewId-1};
            orient = orient*vSet.Views.Orientation{viewId-1};
            indexPairs = indexPairs(inlierIdx, :);
%             [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences_liu(vSet,...
%                 cameraParams, indexPairs, currPoints);
%             warningstate = warning('off','vision:ransac:maxTrialsReached');
%             [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
%                 cameraParams, 'MaxNumTrials', 5000, 'Confidence', 99.99, ...
%                 'MaxReprojectionError', 0.8);
%             warning(warningstate)
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
                'Location', loc);
            vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
            if mod(viewId, 7) == 0
                % Find point tracks in the last 15 views and triangulate.
                windowSize = 15;
                startFrame = max(1, viewId - windowSize);
                tracks = findTracks(vSet, startFrame:viewId);
                camPoses = poses(vSet, startFrame:viewId);
                [xyzPoints, reprojErrors] = triangulateMultiview(tracks, camPoses, ...
                    cameraParams);
                fixedIds = [startFrame, startFrame+1];
                idx = reprojErrors < 2;
                [~, camPoses] = bundleAdjustment(xyzPoints(idx, :), tracks(idx), ...
                    camPoses, cameraParams, 'FixedViewIDs', fixedIds, ...
                    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-9,...
                    'RelativeTolerance', 1e-9, 'MaxIterations', 300);
                vSet = updateView(vSet, camPoses);
            end
            vSet = helperNormalizeViewSet(vSet, groundTruthPoses);
    end
    Diff_EstimatedActual(viewId) = norm(vSet.Views.Location{viewId}-...
        groundTruthPoses.Location{viewId});
    
    %angle(roll,pitch,yaw) from orientation is oppisite to actual angle
    estimatedRM = vSet.Views.Orientation{viewId};
    [estimatedroll,estimatedpitch,estimatedyaw] = RotationMatrix2EulerAngle(estimatedRM);
    actualRM = groundTruthPoses.Orientation{viewId};
    [actualroll,actualpitch,actualyaw] = RotationMatrix2EulerAngle(actualRM);
    if abs(abs(estimatedroll-actualroll)-2*pi) < 5
        estimatedroll = -estimatedroll;
    end
    if abs(abs(estimatedpitch-actualpitch)-2*pi) < 5
        estimatedpitch = -estimatedpitch;
    end
    if abs(abs(estimatedyaw-actualyaw)-2*pi) < 5
        estimatedyaw = -estimatedyaw;
    end
    Diff_EstimatedActual_O(viewId,:) = rad2deg([(estimatedroll-actualroll);...
        (estimatedpitch-actualpitch);(estimatedyaw-actualyaw)]);
    if viewId == 1
        Distance(viewId) = 0;
        TranslationError(viewId) = 0;
        RotationError(viewId,:) = 0;
    else
%         Distance(viewId) = Distance(viewId-1)+...
%             norm(vSet.Views.Location{viewId}-vSet.Views.Location{viewId-1});
        Distance(viewId) = Distance(viewId-1)+...
            norm(groundTruthPoses.Location{viewId}-groundTruthPoses.Location{viewId-1});
        TranslationError(viewId) = Diff_EstimatedActual(viewId)/Distance(viewId);
        RotationError(viewId,:) = Diff_EstimatedActual_O(viewId,:)/Distance(viewId);
    end   
    
    prevPoints   = currPoints;
    preFeatures = currFeatures;
    
    clc;
    if exist('time','var')
        fprintf("\nVOFisheye_2D2D_%03d processing...",time);
    end
    fprintf("\nTotal Complete - %.2f %%",(ind_pose/Npose)*100);
end; clear ind_pose

if exist('time','var')
    filename = ['./results/test_fisheye_2D2D_' num2str(time,'%03d') '.mat'];
    save(filename,'Diff_EstimatedActual','Diff_EstimatedActual_O','vSet',...
        'groundTruthPoses','Distance','TranslationError','RotationError')
else
    filename = './results/test_fisheye_2D2D.mat';
    save(filename)
end

end
