function [UV_fisheye] = FisheyeDistortion(XYZ,Cfisheye,C)

% Fisheye Camera Parameters: C=[x0,y0,z0,a,b,c,a0,a2,a3,a4,cx,cy,s1,s2,s3,s4,Nu,Nv];
% x0,y0,z0   : (1-3)   location (in meters)
% a,b,c      : (4-6)   orientation (in radian): rotation around the camera axes to go from world to camera
% a0,a2,a3,a4: (7-10)  mapping coefficeients: Scaramuzza's Taylor model for projection function
% cx,cy      : (11-12) center of distortion (in pixels)
% s1,s2,s3,s4: (13-16) stretch matrix: 2-by-2 transformation matrix from sensor plane to camera image plane
% Nu,Nv      : (17-18) Number of pixels

%% Check if point is in front of camera
%  by checking the sign of the dot-product between the relative position
%  and the forward vector - vector between point and optical center
vector_OM = XYZ(1:3,:)-kron(Cfisheye(1:3)',ones(1,size(XYZ,2)));
%   - unit vector pointing toward z-axis of camera frame
C2W = CameraToWorld(Cfisheye);
vector_z_cam = C2W*[0;0;1;1]-Cfisheye(1:3)';
%   - sign of dot product, converted to provide 0s and 1s
isInFrontOfCamera = 0.5*(1+sign(dot(vector_OM,kron(vector_z_cam,ones(1,size(XYZ,2))))));
ind_notkeep = isInFrontOfCamera==0;

%%
% Perspective Camera
WC = WorldToCamera(Cfisheye);
XcYcZc = WC*XYZ;

% Fisheye Distortion
UV = ones(size(XcYcZc));
UV(1:3,ind_notkeep) = nan;
UV_perspective = ones(size(XcYcZc));
time(size(XcYcZc,2),1) = 0;
error(size(XcYcZc,2),1) = 0;
for ind_points = 1:size(XcYcZc,2)
    run = 1;
    Xc = XcYcZc(1,ind_points);
    Yc = XcYcZc(2,ind_points);
    Zc = XcYcZc(3,ind_points);
    
    f=C(7);ku=C(10);kv=C(11);
    UV_perspective(1,ind_points) = Xc/Zc*(ku*f);
    UV_perspective(2,ind_points) = Yc/Zc*(kv*f);
    UV_perspective(3,ind_points) = 1;
    if isInFrontOfCamera(ind_points)
        %% Set up limitation to inital chosen points from the surrounding
        %     distance2XcYcZc = vecnorm(XcYcZc-XcYcZc(:,ind_points));
        %     distance2XcYcZc_uv = distance2XcYcZc(1:(ind_points-1));
        %     chance = 2;
        %     k = 3; %number of closest points
        %     [~,CPindex] = mink(distance2XcYcZc_uv,k);
        %
        %     smallerX = find(XcYcZc(1,1:ind_points-1)<XcYcZc(1,ind_points));
        %     [~,smallestX] = max(XcYcZc(1,smallerX));
        %     largerX = find(XcYcZc(1,1:ind_points-1)>XcYcZc(1,ind_points));
        %     [~,largestX] = min(XcYcZc(1,largerX));
        %     checkX = ~isempty(smallerX) && ~isempty(largerX);
        %     smallerY = find(XcYcZc(2,1:ind_points-1)<XcYcZc(2,ind_points));
        %     [~,smallestY] = max(XcYcZc(2,smallerY));
        %     largerY = find(XcYcZc(2,1:ind_points-1)>XcYcZc(2,ind_points));
        %     [~,largestY] = min(XcYcZc(2,largerY));
        %     checkY = ~isempty(smallerY) && ~isempty(largerY);
        
        %% different scenario for Xc,Yc (3 if end)
        if run && abs(Xc) >= 1e-4 %case Yc ~= 0
            run = 0; error1 = 1; error2 = -1; error_uv = 1; % lambda1 = -1; lambda2 = -1;
            %InitialNewtonIteration (find range)
            while error1 > 0 || error2 < 0 % || lambda1 < 0 || lambda2 < 0
                %             u1 = (rand-0.5)*10000; u2 = (rand-0.5)*10000;
                %             if length(CPindex) == k
                %                 u1 = UV(1,CPindex(randi(k)))+(rand-0.5)*200;
                %                 u2 = UV(1,CPindex(randi(k)))+(rand-0.5)*200;
                %             end
                %             if checkX && chance
                %                 if chance == 2
                %                     u1  = UV(1,smallerX(smallestX));
                %                     u2  = UV(1,largerX(largestX));
                %                 else
                %                     u1 = UV(1,largerX(largestX));
                %                     u2 = UV(1,smallerX(smallestX));
                %                 end
                %                 chance = chance-1;
                %             end
                u1 = rand*UV_perspective(1,ind_points);
                u2 = rand*UV_perspective(1,ind_points);
                v1 = Yc/Xc*u1; p1 = sqrt(u1^2+v1^2);
                F1 = Cfisheye(7)+Cfisheye(8)*p1^2+Cfisheye(9)*p1^3+Cfisheye(10)*p1^4;
                error1 = Xc*F1-u1*Zc;
                v2 = Yc/Xc*u2; p2 = sqrt(u2^2+v2^2);
                F2 = Cfisheye(7)+Cfisheye(8)*p2^2+Cfisheye(9)*p2^3+Cfisheye(10)*p2^4;
                error2 = Xc*F2-u2*Zc;
                %                lambda1 = Xc/u1; lambda2 = Xc/u2;
            end;% clear v1 p1 F1 v2 p2 F2
            %NewtonIteration
            while abs(error_uv) > 1e-8
                time(ind_points) = time(ind_points)+1;
                if time(ind_points)>2e2; break; end
                u = u1+(u2-u1)*(0-error1)/(error2-error1);
                v = Yc/Xc*u;
                p = sqrt(u^2+v^2);
                F = Cfisheye(7)+Cfisheye(8)*p^2+Cfisheye(9)*p^3+Cfisheye(10)*p^4;
                error_uv = Xc*F-u*Zc;
                if error_uv < 0
                    u1 = u;
                    error1 = error_uv;
                else
                    u2 = u;
                    error2 = error_uv;
                end
            end
        end
        if run && abs(Yc) >= 1e-4 %case Xc ~= 0
            run = 0; error1 = 1; error2 = -1; error_uv = 1; % lambda1 = -1; lambda2 = -1;
            %InitialNewtonIteration (find range)
            while error1 > 0 || error2 < 0 % || lambda1 < 0 || lambda2 < 0
                %             v1 = (rand-0.5)*10000; v2 = (rand-0.5)*10000;
                %             if length(CPindex) == k
                %                 v1 = UV(2,CPindex(randi(k)))+(rand-0.5)*200;
                %                 v2 = UV(2,CPindex(randi(k)))+(rand-0.5)*200;
                %             end
                %             if checkY && chance
                %                 if chance == 2
                %                     v1  = UV(2,smallerY(smallestY));
                %                     v2  = UV(2,largerY(largestY));
                %                 else
                %                     v1 = UV(2,largerY(largestY));
                %                     v2 = UV(2,smallerY(smallestY));
                %                 end
                %                 chance = chance-1;
                %             end
                v1 = rand*UV_perspective(2,ind_points);
                v2 = rand*UV_perspective(2,ind_points);
                u1 = Xc/Yc*v1; p1 = sqrt(u1^2+v1^2);
                F1 = Cfisheye(7)+Cfisheye(8)*p1^2+Cfisheye(9)*p1^3+Cfisheye(10)*p1^4;
                error1 = Yc*F1-v1*Zc;
                u2 = Xc/Yc*v2; p2 = sqrt(u2^2+v2^2);
                F2 = Cfisheye(7)+Cfisheye(8)*p2^2+Cfisheye(9)*p2^3+Cfisheye(10)*p2^4;
                error2 = Yc*F2-v2*Zc;
                %                lambda1 = Yc/v1; lambda2 = Yc/v2;
            end;% clear u1 p1 F1 u2 p2 F2
            %NewtonIteration
            while abs(error_uv) > 1e-8
                time(ind_points) = time(ind_points)+1;
                if time(ind_points)>2e2; break; end
                v = v1+(v2-v1)*(0-error1)/(error2-error1);
                u = Xc/Yc*v;
                p = sqrt(u^2+v^2);
                F = Cfisheye(7)+Cfisheye(8)*p^2+Cfisheye(9)*p^3+Cfisheye(10)*p^4;
                error_uv = Yc*F-v*Zc;
                if error_uv < 0
                    v1 = v;
                    error1 = error_uv;
                else
                    v2 = v;
                    error2 = error_uv;
                end
            end
        end
        if run && abs(Xc) <= 1e4 && abs(Yc) <= 1e-4 %case Xc & Yc = 0
            run = 0; u = 0; v = 0;
        end
        %%
        UV(1,ind_points) = u;
        UV(2,ind_points) = v;
        UV(3,ind_points) = F;
        error(ind_points) = error_uv;
        if abs(error_uv)>=1
            UV(:,ind_points) = nan;
        end
    end
end; clear ind_points

%% affine transformation
StretchMatrix = [Cfisheye(13) Cfisheye(14);Cfisheye(15) Cfisheye(16)];
DistortionCenter = [Cfisheye(11);Cfisheye(12)];
for ind_points = 1:size(UV,2)
    if isInFrontOfCamera(ind_points)
        UV(1:2,ind_points) = StretchMatrix*UV(1:2,ind_points)+DistortionCenter;
        UV(3,ind_points) = 1;
    end
end; clear ind_points

UV_fisheye = UV;

end

