%% Draw an image plane
% draws a rectangle that represents the image plane. It draws the center of
% the camera too.

% C:     vector of the camera parameters: C=[x0,y0,z0,a,b,g,f,u0,v0,ku,kv,Nu,Nv]


function DrawImagePlane(C)
    %Draw camera origin
    plot3(C(1),C(2),C(3),'o');
    plot3(C(1),C(2),C(3),'+');

    % homogeneous coordinates of 4 corners of the image plane expressed in the camera frame
    p(1,1)=-C(12)/2/C(10); p(2,1)=-C(13)/2/C(11); p(3,1)=C(7); p(4,1)=1;
    p(1,2)=-C(12)/2/C(10); p(2,2)=+C(13)/2/C(11); p(3,2)=C(7); p(4,2)=1;
    p(1,3)=+C(12)/2/C(10); p(2,3)=+C(13)/2/C(11); p(3,3)=C(7); p(4,3)=1;
    p(1,4)=+C(12)/2/C(10); p(2,4)=-C(13)/2/C(11); p(3,4)=C(7); p(4,4)=1;

    % CW: Camera to World transformation
    CW = CameraToWorld(C);

    % transform co-ordinates to world co-ordinates
    P(:,1)=CW*p(:,1);
    P(:,2)=CW*p(:,2);
    P(:,3)=CW*p(:,3);
    P(:,4)=CW*p(:,4);

    %draw image plane
    patch(P(1,:),P(2,:),P(3,:),[.9,.9,1],'FaceAlpha',0.2,'FaceColor',[0 1 1]);

end