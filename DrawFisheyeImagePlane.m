%% Draw an image plane (Fisheye)
% draws a rectangle that represents the perspective image plane.
% draws a circle that represents the fisheye image plane.
% It draws the center of the camera too.

% C:     vector of the camera parameters: C=[x0,y0,z0,a,b,g,f,u0,v0,ku,kv,Nu,Nv]


function DrawFisheyeImagePlane(C,UV,XYZ)
%Draw camera origin
plot3(C(1),C(2),C(3),'o');
plot3(C(1),C(2),C(3),'+');

% homogeneous coordinates of 4 corners of the image plane expressed in the camera frame
p(1,1)=-C(12)/2/C(10); p(2,1)=-C(13)/2/C(11); p(3,1)=-C(7); p(4,1)=1;
p(1,2)=-C(12)/2/C(10); p(2,2)=+C(13)/2/C(11); p(3,2)=-C(7); p(4,2)=1;
p(1,3)=+C(12)/2/C(10); p(2,3)=+C(13)/2/C(11); p(3,3)=-C(7); p(4,3)=1;
p(1,4)=+C(12)/2/C(10); p(2,4)=-C(13)/2/C(11); p(3,4)=-C(7); p(4,4)=1;

% CW: Camera to World transformation
CW = CameraToWorld(C);

% transform co-ordinates to world co-ordinates
P(:,1)=CW*p(:,1);
P(:,2)=CW*p(:,2);
P(:,3)=CW*p(:,3);
P(:,4)=CW*p(:,4);

%draw image plane
patch(P(1,:),P(2,:),P(3,:),[.9,.9,1],'FaceAlpha',0.2,'FaceColor',[0 1 1]);
hold on

%draw fisheye circle
MaxAngleX = rad2deg(atan2(C(12)/2/C(10),C(7)));
MaxAngleY = rad2deg(atan2(C(13)/2/C(11),C(7)));
for i = 1:size(XYZ,2)
    rx(i) = MaxAngleX*sqrt((UV(1,i)-C(8))^2+(UV(2,i)-C(9))^2)...
        /rad2deg(atan2(sqrt(XYZ(1,i)^2+XYZ(2,i)^2),XYZ(3,i)));
    ry(i) = MaxAngleY*sqrt((UV(1,i)-C(8))^2+(UV(2,i)-C(9))^2)...
        /rad2deg(atan2(sqrt(XYZ(1,i)^2+XYZ(2,i)^2),XYZ(3,i)));
end; clear i
center = [0;0;-C(7);1];
Center = CW*center;
RX = rx(1)*abs(P(1,1))/(C(12)/2);
RY = ry(1)*abs(P(2,1))/(C(13)/2);
ellipse3D(Center(1),Center(2),Center(3),RX,RY,[0 1 1],0.5);

end