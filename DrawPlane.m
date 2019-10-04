function DrawPlane(planePoint,planeNormal)

x1 = planePoint(1); y1 = planePoint(2); z1 = planePoint(3);
w = null(planeNormal); % Find two orthonormal vectors which are orthogonal to v
[P,Q] = meshgrid(-50:50); % Provide a gridwork (you choose the size)
X = x1+w(1,1)*P+w(1,2)*Q; % Compute the corresponding cartesian coordinates
Y = y1+w(2,1)*P+w(2,2)*Q; %   using the two vectors in w
Z = z1+w(3,1)*P+w(3,2)*Q;
hold all; surf(X,Y,Z)

end