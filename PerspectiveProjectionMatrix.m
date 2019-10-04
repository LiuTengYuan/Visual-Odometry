%% Obtain the projection matrix parameters from the camera position

function M=PerspectiveProjectionMatrix(C)
    % World to camera
    WC = WorldToCamera(C);

    %Projection point in the image
    F=[C(7)     0       0
        0       C(7)    0 
        0       0       1];

    %Distance units to pixels
    K=[C(10)    0       C(8)
        0       C(11)   C(9)
        0       0       1];

    %Projection matrix
    M=K*F*WC;
end
