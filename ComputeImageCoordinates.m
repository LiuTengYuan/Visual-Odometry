function [UV,ind_visible] = ComputeImageCoordinates(C,XYZ)
% Compute the image coordinates of points XYZ, seen by camera C.
% INPUTS:
%   - C     - 1 x 13      - Camera parameters
%   - XYZ   - 4 x Npoints - World homogeneous coordinates of points
% OUTPUTS:
%   - UV          - 2 x Nvisible    - Image coordinates of the visible points
%   - ind_visible - 1 x Nvisible    - Index of the visible XYZ points

% Perspective Projection matrix
Pmat = PerspectiveProjectionMatrix(C);

% Project into image frame, in pixels
UV = Pmat*XYZ;

% Normalize by the 3rd homogeneous coordintate
for ind_points = 1:size(UV,2);
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
ind_visible = find(isInsideImage.*isInFrontOfCamera);

