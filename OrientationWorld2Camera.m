function RM=OrientationWorld2Camera(a_v,b_v,c_v)
%create rotation matrix(orientation) from world to camera

RM = cell(length(a_v),1);
for i = 1:length(a_v)
    % Rotation
    Rx=[1       0         0
        0   cos(a_v(i))  sin(a_v(i))
        0   -sin(a_v(i))  cos(a_v(i))];
    
    Ry=[cos(b_v(i))    0     -sin(b_v(i))
        0         1       0
        sin(b_v(i))   0     cos(b_v(i))];
    
    Rz=[cos(c_v(i))  sin(c_v(i))   0
        -sin(c_v(i))  cos(c_v(i))   0
        0          0        1];    
    
    RM(i)={Rx*Ry*Rz};
end

end