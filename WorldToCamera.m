% Compute matrix that transforms co-ordinates from the world frame to the camera frame

function WC=WorldToCamera(C)
    %Rotation
    Rx=[1   0   0
        0   cos(C(4))   sin(C(4))
        0   -sin(C(4))   cos(C(4))];

    Ry=[cos(C(5))  0  -sin(C(5))
            0      1       0        
        sin(C(5))  0  cos(C(5))];
    
    Rz=[cos(C(6))   sin(C(6)) 0
        -sin(C(6))  cos(C(6)) 0
            0           0    1];

    WC=(Rx*Ry*Rz);
    
    %Translation
    T=[-C(1) -C(2) -C(3)]';

    %Transformation 
    Tp=WC*T;         %translation
    WC(:,4)=Tp;      %compose homogeneous form
end