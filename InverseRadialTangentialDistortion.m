function [UV] = InverseRadialTangentialDistortion(UV,cameraParams)

%Reference
%https://www.mathworks.com/help/symbolic/examples/developing-an-algorithm-for-undistorting-an-image.html
%http://www.vision.caltech.edu/bouguetj/calib_doc/htmls/parameters.html

% Radio Distortion: Rd = [k1,k2,k3]
% x_distorted = x*(1+k1*r^2+k2*r^4+k3*r^6)
% y_distorted = y*(1+k1*r^2+k2*r^4+k3*r^6)
% r^2 = x^2+y^2
% (x,y): undistored pixel locations, x and y are in normalized image
% coordinates, which are cacualted from pixel coordinates by translating to
% the optical center and divided by the focal length in pixels. Thus x and
% y are dimensionless
% (k1,k2,k3): radio distortion coefficients

% Tangential Distortion: Td = [p1 p2]
% x_distorted = x+[2*p1*x*y+p2*(r^2+2*x^2)]
% y_distorted = y+[p1*(r^2+2*y^2)+2*p2*x*y]
% r^2 = x^2+y^2
% (x,y): undistored pixel locations, x and y are in normalized image
% coordinates, which are cacualted from pixel coordinates by translating to
% the optical center and divided by the focal length in pixels. Thus x and
% y are dimensionless
% (p1,p2): tangential distortion coefficients


CP = cameraParams.IntrinsicMatrix'; 
%Normalized image coordinates
for ind_points = 1:size(UV,2)
    UV(:,ind_points) = inv(CP)*UV(:,ind_points);
end; clear ind_points
%Inverse Radio & Tangential Distortion
for ind_points = 1:size(UV,2)
    syms k_1 k_2 k_3 p_1 p_2 real
    syms r x y x_d y_d
    eq1 = x_d == subs(x*(1+k_1*r^2+k_2*r^4+k_3*r^6)+2*p_1*x*y+p_2*(r^2+2*x^2), r, sqrt(x^2+y^2));
    eq2 = y_d == subs(y*(1+k_1*r^2+k_2*r^4+k_3*r^6)+2*p_2*x*y+p_1*(r^2+2*y^2), r, sqrt(x^2+y^2));
    parameters = [x_d y_d k_1 k_2 k_3 p_1 p_2];
    parameterValues = [UV(1:2,ind_points)' cameraParams.RadialDistortion cameraParams.TangentialDistortion];
    eq1 = expand(subs(eq1, parameters, parameterValues));
    eq2 = expand(subs(eq2, parameters, parameterValues));
    Result = solve([eq1, eq2], [x,y],'Real',true,'ReturnConditions',true);
    UV(1:2,ind_points) = [Result.x;Result.y];
end; clear ind_points
%De-Normalized image coordinates
for ind_points = 1:size(UV,2)
    UV(:,ind_points) = CP*UV(:,ind_points);
end; clear ind_points

end