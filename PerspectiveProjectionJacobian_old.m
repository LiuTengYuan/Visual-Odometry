%% Computation of the Jacobian matrix of the Perspective Projection Matrix for a pinhole camera

function [J_pos,J_att,J_lmk] = PerspectiveProjectionJacobian_old(C,XYZ)
    % camera parameters
    x_cam = C(1); y_cam = C(2); z_cam = C(3);
    a = C(4); b = C(5); c = C(6);
    f = C(7);
    ku = C(10); kv = C(11);
    
    % landmark position
    x_lmk = XYZ(1); y_lmk = XYZ(2); z_lmk = XYZ(3);
    
    % intermediate notations
    ca = cos(a); cb = cos(b); cc = cos(c);
    sa = sin(a); sb = sin(b); sc = sin(c);
    
    R11 = cb*cc;          R12 = cb*sc;          R13 = -sb;
    R21 = sa*sb*cc-ca*sc; R22 = sa*sb*sc+ca*cc; R23 = sa*cb;
    R31 = ca*sb*cc+sa*sc; R32 = ca*sb*sc-sa*cc; R33 = ca*cb;
        
    lambda = R31*x_lmk + R32*y_lmk + R33*z_lmk - z_cam;
    
    % Jacobian wrt camera position
    J_pos = [-f*ku/lambda, 0, f*ku/lambda^2*(R11*x_lmk + R12*y_lmk + R13*z_lmk - x_cam);
             0, -f*kv/lambda, f*kv/lambda^2*(R21*x_lmk + R22*y_lmk + R23*z_lmk - y_cam)];
    
    % Jacobian wrt camera attitude
    J_att(1,1) = f*ku/lambda^2*(-(R11*x_lmk+R12*y_lmk+R13*z_lmk-x_cam)*(x_lmk*(ca*sc-sa*sb*cc)+y_lmk*(-ca*cc-sa*sb*sc)-z_lmk*sa*cb));
    J_att(1,2) = f*ku/lambda^2*((-x_lmk*sb*cc - y_lmk*sb*sc - z_lmk*cb)*lambda - (R11*x_lmk+R12*y_lmk+R13*z_lmk-x_cam)*(x_lmk*ca*cb*cc+y_lmk*(-ca*cc-sa*sb*sc)-z_lmk*sa*cb));
    J_att(1,3) = f*ku/lambda^2*((-x_lmk*cb*sc + y_lmk*cb*cc)*lambda - (R11*x_lmk+R12*y_lmk+R13*z_lmk-x_cam)*(x_lmk*(sa*cc-ca*sb*sc)+y_lmk*(sa*sc+ca*sb*cc)));
    J_att(2,1) = f*kv/lambda^2*((x_lmk*(sa*sc+ca*sb*cc) + y_lmk*(-sa*cc+ca*sb*sc) + z_lmk*ca*cb)*lambda - (R21*x_lmk+R22*y_lmk+R23*z_lmk-y_cam)*(x_lmk*(ca*sc-sa*sb*cc)+y_lmk*(-ca*cc-sa*sb*sc)-z_lmk*sa*cb));
    J_att(2,2) = f*kv/lambda^2*((-x_lmk*sb*sc + y_lmk*sa*cb*sc + z_lmk*ca*cb*sc)*lambda - (R21*x_lmk+R22*y_lmk+R23*z_lmk-y_cam)*(-x_lmk*cb-y_lmk*sa*sb-z_lmk*ca*sb));
    J_att(2,3) = f*kv/lambda^2*((x_lmk*cb*cc + y_lmk*(sa*sb*cc-ca*sc) + z_lmk*(ca*sb*cc+sa*sc))*lambda - (R21*x_lmk+R22*y_lmk+R23*z_lmk-y_cam)*z_lmk*ca*cb);
    
    % Jacobian wrt landmark position
    J_lmk(1,1) = f*ku/lambda^2*[R11*lambda - R31*(R11*x_lmk+R12*y_lmk+R13*z_lmk-x_cam)];
    J_lmk(1,2) = f*ku/lambda^2*[R12*lambda - R32*(R11*x_lmk+R12*y_lmk+R13*z_lmk-x_cam)];
    J_lmk(1,3) = f*ku/lambda^2*[R13*lambda - R33*(R11*x_lmk+R12*y_lmk+R13*z_lmk-x_cam)];
    J_lmk(2,1) = f*kv/lambda^2*[R21*lambda - R31*(R21*x_lmk+R22*y_lmk+R23*z_lmk-y_cam)];
    J_lmk(2,2) = f*kv/lambda^2*[R22*lambda - R32*(R21*x_lmk+R22*y_lmk+R23*z_lmk-y_cam)];
    J_lmk(2,3) = f*kv/lambda^2*[R23*lambda - R33*(R21*x_lmk+R22*y_lmk+R23*z_lmk-y_cam)];
end
