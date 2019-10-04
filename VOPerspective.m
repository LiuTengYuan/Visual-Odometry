function VOPerspective(time)

%Output #4: VO script using the perspective camera simulator + example of VO output on a simple trajectory

clearvars -except time Ntimes; close all; clc; tic;

if exist('time','var')
    filename = ['./results/test_initialcondition_' num2str(time,'%03d') '.mat'];
    load(filename)
else
    filename = './results/test_initialcondition.mat';
    load(filename)
end

%% Create the camera parameters object using camera intrinsics
f=0.05;                    %focal length (in m)
u0=1280/2; v0=720/2;       %optical center (in pixels)
ku=1/23e-6; kv=1/23e-6;    %inverse of pixel size (in pixels/m)
Nu=1280; Nv=720;           %Number of pixels
Rd = [0 0 0];        %Radio Distortion Coefficients
Td = [0 0];                %Tangential Distortion Coefficients
K = [f*ku 0 u0; 0 f*kv v0; 0 0 1]'; %Intrinsic Matrix
cameraParams = cameraParameters('IntrinsicMatrix', K, 'RadialDistortion', Rd, 'TangentialDistortion', Td,...
    'ImageSize', [Nv Nu], 'NumRadialDistortionCoefficients', 3, 'EstimateTangentialDistortion', true);

%%
vSet = viewSet;
ind_keep = cell(Npose,1);
C = zeros(Npose,13);
XYZ = cell(Npose,1);
XYZP = cell(Npose,1);
UV = cell(Npose,1);

Diff_EstimatedActual = NaN(height(groundTruthPoses),1); %(m)
Diff_EstimatedActual_O = NaN(height(groundTruthPoses),3); %(deg)
Distance = NaN(height(groundTruthPoses),1); %(m)
TranslationError = NaN(height(groundTruthPoses),1); %(%)
RotationError = NaN(height(groundTruthPoses),3); %(deg/m)

for ind_pose = 1:Npose
    %% Camera Parameters: C=[x0,y0,z0,a,b,c,f,u0,v0,ku,kv,Nu,Nv];
    % x0,y0,z0: (1-3)   location (in meters)
    % a,b,c   : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
    % f       : (7)     focal length (in m)
    % u0,v0   : (8-9)   optical center (in pixels)
    % ku,kv   : (10-11) inverse of pixel size (in pixels/m)
    % Nu,Nv   : (12-13) Number of pixels
    
    C(ind_pose,:)=[x0_v(ind_pose),y0_v(ind_pose),z0_v(ind_pose),...
        a_v(ind_pose),b_v(ind_pose),c_v(ind_pose),...
        f,... %0.025,...
        u0,v0,...
        ku,kv,...
        Nu,Nv];
    
    %% Project points onto the image plane
    % Perspective Projection matrix
    P = PerspectiveProjectionMatrix(C(ind_pose,:));
    
    %Road Scenario(choose world points from segments)
    Segment = ceil(ind_pose/(Npose/segment));
    if Segment > 1 && Segment < segment
        XYZ{ind_pose} = [XYZ_segment{Segment-1},XYZ_segment{Segment},XYZ_segment{Segment+1}];
    elseif Segment == 1
        XYZ{ind_pose} = [XYZ_segment{Segment},XYZ_segment{Segment+1}];
    else %Segment = segment
        XYZ{ind_pose} = [XYZ_segment{Segment-1},XYZ_segment{Segment}];
    end
    
    %Road Scenario(block world points penertating buildings)
    currPos = [x0_v(ind_pose) y0_v(ind_pose) z0_v(ind_pose)];
    XYZP{ind_pose} = CheckPenetratePoints(currPos, XYZ{ind_pose},...
        XYZ_LBuilding{Segment}, XYZ_RBuilding{Segment});
    
    % Project into image frame, in pixels
    UV(ind_pose) = {P*XYZP{ind_pose}};
    
    % Normalize by the 3rd homogeneous coordintate
    for ind_points = 1:size(UV{ind_pose},2)
        UV{ind_pose}(:,ind_points) = UV{ind_pose}(:,ind_points)/UV{ind_pose}(3,ind_points);
    end; clear ind_points
    
    % Exclude points outside the image
    isInsideImage = ( UV{ind_pose}(1,:) <= C(ind_pose,12) ).*( UV{ind_pose}(1,:) > 0 )...
        .*( UV{ind_pose}(2,:) <= C(ind_pose,13) ).*( UV{ind_pose}(2,:) > 0);
    % Check if point is in front of camera, by checking the sign of the
    % dot-product between the relative position and the forward vector
    %   - vector between point and optical center
    vector_OM = XYZP{ind_pose}(1:3,:)-kron(C(ind_pose,1:3)',ones(1,size(XYZP{ind_pose},2)));
    %   - unit vector pointing toward z-axis of camera frame
    C2W = CameraToWorld(C(ind_pose,:));
    vector_z_cam = C2W*[0;0;1;1]-C(ind_pose,1:3)';
    %   - sign of dot product, converted to provide 0s and 1s
    isInFrontOfCamera = 0.5*(1+sign( dot(vector_OM,kron(vector_z_cam,ones(1,size(XYZP{ind_pose},2))) ) ));
    %   - indexes of points that appear in the image
    ind_keep{ind_pose} = find(isInsideImage.*isInFrontOfCamera);
    
    %% Visual Odometry
    
    viewId = ind_pose;
    
    % undistortImage=InverseRadialTangentialDistortion?????????????????????
    %InverseRadialTangentialDistortion could not solve problem with high
    %order x & y???????????????????????????????????????????????????????????
    %     UV(:,ind_keep) = InverseRadialTangentialDistortion(UV(:,ind_keep),cameraParams);
    
    PerspectivePoints = UV{ind_pose}(1:2,ind_keep{ind_pose})';
    
    %select uniform fisheye points
    numPoints = height(groundTruthPoses);
    SURFPerspectivePoints = SURFPoints(PerspectivePoints);
    uniformPerspectivePoints = selectUniform(SURFPerspectivePoints,numPoints,[C(ind_pose,13) C(ind_pose,12)]);
    PerspectivecurrPoints = uniformPerspectivePoints.Location;
    indexPairsUniform = DetectandMatchFeatures(PerspectivecurrPoints',single(PerspectivePoints)');
    
    currPoints = PerspectivecurrPoints;
    currFeatures = XYZP{ind_pose}(1:3,ind_keep{ind_pose}(indexPairsUniform(:,2)));
    
    switch ind_pose
        case 1
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', ...
                groundTruthPoses.Orientation{1}, 'Location', groundTruthPoses.Location{1});
        case 2
            indexPairs = DetectandMatchFeatures(preFeatures,currFeatures);
            [orient, loc, inlierIdx] = helperEstimateRelativePose(...
                prevPoints(indexPairs(:,1),:), currPoints(indexPairs(:,2),:), cameraParams);
            %             loc = loc+vSet.Views.Location{1};
            %             orient = orient*vSet.Views.Orientation{1};
            indexPairs = indexPairs(inlierIdx, :);
            vSet = addView(vSet, viewId, 'Points', currPoints, 'Orientation', orient, ...
                'Location', loc);
            vSet = addConnection(vSet, viewId-1, viewId, 'Matches', indexPairs);
            vSet = helperNormalizeViewSet(vSet, groundTruthPoses);
        case num2cell(3:15)
            indexPairs = DetectandMatchFeatures(preFeatures,currFeatures);
            [~, ~, inlierIdx] = helperEstimateRelativePose(...
                prevPoints(indexPairs(:,1),:), currPoints(indexPairs(:,2),:), cameraParams);
            indexPairs = indexPairs(inlierIdx, :);
            [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences_liu(vSet,...
                cameraParams, indexPairs, currPoints);
            warningstate = warning('off','vision:ransac:maxTrialsReached');
            [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
                cameraParams, 'Confidence', 99.99, 'MaxReprojectionError', 0.8);
            warning(warningstate)
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
            [worldPoints, imagePoints] = helperFind3Dto2DCorrespondences_liu(vSet,...
                cameraParams, indexPairs, currPoints);
            warningstate = warning('off','vision:ransac:maxTrialsReached');
            [orient, loc] = estimateWorldCameraPose(imagePoints, worldPoints, ...
                cameraParams, 'MaxNumTrials', 5000, 'Confidence', 99.99, ...
                'MaxReprojectionError', 0.8);
            warning(warningstate)
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
    end
    
    Diff_EstimatedActual(viewId) = norm(vSet.Views.Location{viewId}-...
        groundTruthPoses.Location{viewId});
    
    %angle(roll,pitch,yaw) from orientation is oppisite to actual angle
    estimatedRM = vSet.Views.Orientation{viewId};
    [estimatedroll,estimatedpitch,estimatedyaw] = RotationMatrix2EulerAngle...
        (estimatedRM,a_v(viewId),b_v(viewId),c_v(viewId));
    actualRM = groundTruthPoses.Orientation{viewId};
    [actualroll,actualpitch,actualyaw] = RotationMatrix2EulerAngle...
        (actualRM,a_v(viewId),b_v(viewId),c_v(viewId));
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
        RotationError(viewId) = 0;
    else
        Distance(viewId) = Distance(viewId-1)+...
            norm(vSet.Views.Location{viewId}-vSet.Views.Location{viewId-1});
        TranslationError(viewId) = Diff_EstimatedActual(viewId)/Distance(viewId);
        RotationError(viewId,:) = Diff_EstimatedActual_O(viewId,:)/Distance(viewId);
    end
    
    prevPoints   = currPoints;
    preFeatures = currFeatures;

end; clear ind_pose

if exist('time','var')
    filename = ['./results/test_perspective_' num2str(time,'%03d') '.mat'];
    save(filename,'Diff_EstimatedActual','Diff_EstimatedActual_O','vSet',...
        'groundTruthPoses','Distance','TranslationError','RotationError')
else
    filename = './results/test_perspective.mat';
    save(filename)
end


end
