function [UV] = RadialTangentialDistortion(C,UV,Rd,Td)

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
%--------------------------------------------------------------------------
% Tangential Distortion: Td = [p1 p2]
% x_distorted = x+[2*p1*x*y+p2*(r^2+2*x^2)]
% y_distorted = y+[p1*(r^2+2*y^2)+2*p2*x*y]
% r^2 = x^2+y^2
% (x,y): undistored pixel locations, x and y are in normalized image
% coordinates, which are cacualted from pixel coordinates by translating to
% the optical center and divided by the focal length in pixels. Thus x and
% y are dimensionless
% (p1,p2): tangential distortion coefficients

%Projection point in the image
F=[C(7)     0       0
    0       C(7)    0
    0       0       1];
%Distance units to pixels
K=[C(10)    0       C(8)
    0       C(11)   C(9)
    0       0       1];
CP = K*F; 
%Normalized image coordinates
for ind_points = 1:size(UV,2)
    UV(:,ind_points) = inv(CP)*UV(:,ind_points);
end; clear ind_points

%Radio & Tangential Distortion
for ind_points = 1:size(UV,2)
    r = sqrt(UV(1,ind_points)^2+UV(2,ind_points)^2);
    Tangential_Distortion = zeros(2,1);
    Tangential_Distortion(1) = (2*Td(1)*UV(1,ind_points)*UV(2,ind_points)+Td(2)*(r^2+2*UV(1,ind_points)^2));
    Tangential_Distortion(2) = (Td(1)*(r^2+2*UV(2,ind_points)^2)+2*Td(2)*UV(1,ind_points)*UV(2,ind_points));
    UV(1,ind_points) = UV(1,ind_points)*(1+Rd(1)*r^2+Rd(2)*r^4+Rd(3)*r^6)+Tangential_Distortion(1);
    UV(2,ind_points) = UV(2,ind_points)*(1+Rd(1)*r^2+Rd(2)*r^4+Rd(3)*r^6)+Tangential_Distortion(2);
end; clear ind_points

%De-Normalized image coordinates
for ind_points = 1:size(UV,2)
    UV(:,ind_points) = CP*UV(:,ind_points);
end; clear ind_points

end