%% Compute matrix that transforms co-ordinates 
%in the camera frame to the world frame
function [CW]=CameraToWorld(C)

    % Rotation 
    Rx=[1       0         0
        0   cos(C(4))  -sin(C(4))
        0   sin(C(4))  cos(C(4))];

    Ry=[cos(C(5))    0     sin(C(5))
           0         1       0
        -sin(C(5))   0     cos(C(5))];

    Rz=[ cos(C(6))  -sin(C(6))   0
         sin(C(6))  cos(C(6))   0
            0          0        1];
        
    CW=(Rz*Ry*Rx);
    
%     CW = [cos(C(5))*cos(C(6)),sin(C(4))*sin(C(5))*cos(C(6))-cos(C(4))*sin(C(6)),cos(C(4))*sin(C(5))*cos(C(6))+sin(C(4))*sin(C(6));
%           cos(C(5))*sin(C(6)),sin(C(4))*sin(C(5))*sin(C(6))+cos(C(4))*cos(C(6)),cos(C(4))*sin(C(5))*sin(C(6))-sin(C(4))*cos(C(6));
%           -sin(C(5)),sin(C(4))*cos(C(5)),cos(C(4))*cos(C(5))];

    %Translation 
    T=[C(1) C(2) C(3)];

    %Transformation
    CW(:,4)=T;
end