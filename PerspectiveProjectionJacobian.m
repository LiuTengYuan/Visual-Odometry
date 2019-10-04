%% Computation of the Jacobian matrix of the Perspective Projection Matrix for a pinhole camera
% XYZ should be a vector

function [J_pos,J_att,J_lmk] = PerspectiveProjectionJacobian(C,XYZ)
    % camera parameters
    x_cam = C(1); y_cam = C(2); z_cam = C(3);
    a = C(4); b = C(5); c = C(6);
    f = C(7);
    ku = C(10); kv = C(11);
    
    % landmark position
    x_lmk = XYZ(1,1); y_lmk = XYZ(2,1); z_lmk = XYZ(3,1);
    
    % intermediate notations
    dx = x_lmk - x_cam;
    dy = y_lmk - y_cam;
    dz = z_lmk - z_cam;
    
    ca = cos(a); cb = cos(b); cc = cos(c);
    sa = sin(a); sb = sin(b); sc = sin(c);
    
    R11 = cb*cc;          R12 = cb*sc;          R13 = -sb;
    R21 = sa*sb*cc-ca*sc; R22 = sa*sb*sc+ca*cc; R23 = sa*cb;
    R31 = ca*sb*cc+sa*sc; R32 = ca*sb*sc-sa*cc; R33 = ca*cb;
    
    R21a = sa*sc+ca*sb*cc; R22a = -sa*cc+ca*sb*sc; R23a = ca*cb;
    R31a = ca*sc-sa*sb*cc; R32a = -ca*cc-sa*sb*sc; R33a = -sa*cb;
    
    R11b = -sb*cc; R12b = -sb*sc; R13b = -cb;
    R21b = sa*cb*cc; R22b = sa*cb*sc; R23b = -sa*sb;
    R31b = ca*cb*cc; R32b = ca*cb*sc; R33b = -ca*sb;
    
    R11c = -cb*sc; R12c = cb*cc;
    R21c = -ca*cc-sa*sb*sc; R22c = -ca*sc+sa*sb*cc;
    R31c = sa*cc-ca*sb*sc; R32c = sa*sc+ca*sb*cc;
    
    alpha  = R11*dx + R12*dy + R13*dz;
    beta   = R21*dx + R22*dy + R23*dz;
    lambda = R31*dx + R32*dy + R33*dz;
    
    % Jacobian wrt camera position
    J_pos(1,1) = f*ku/lambda^2*(-R11*lambda+R31*alpha);
    J_pos(1,2) = f*ku/lambda^2*(-R12*lambda+R32*alpha);
    J_pos(1,3) = f*ku/lambda^2*(-R13*lambda+R33*alpha);
    J_pos(2,1) = f*kv/lambda^2*(-R21*lambda+R31*beta);
    J_pos(2,2) = f*kv/lambda^2*(-R22*lambda+R32*beta);
    J_pos(2,3) = f*kv/lambda^2*(-R23*lambda+R33*beta);
    
    % Jacobian wrt camera attitude
    J_att(1,1) = f*ku/lambda^2*(                                -alpha*(R31a*dx+R32a*dy+R33a*dz));
    J_att(1,2) = f*ku/lambda^2*((R11b*dx+R12b*dy+R13b*dz)*lambda-alpha*(R31b*dx+R32b*dy+R33b*dz));
    J_att(1,3) = f*ku/lambda^2*((        R11c*dx+R12c*dy)*lambda-alpha*(R31c*dx+R32c*dy));
    J_att(2,1) = f*kv/lambda^2*((R21a*dx+R22a*dy+R23a*dz)*lambda-beta*(R31a*dx+R32a*dy+R33a*dz));
    J_att(2,2) = f*kv/lambda^2*((R21b*dx+R22b*dy+R23b*dz)*lambda-beta*(R31b*dx+R32b*dy+R33b*dz));
    J_att(2,3) = f*kv/lambda^2*((R21c*dx+R22c*dy)*lambda-beta*(R31c*dx+R32c*dy));
    
    % Jacobian wrt landmark position
    J_lmk(1,1) = f*ku/lambda^2*(R11*lambda - R31*alpha);
    J_lmk(1,2) = f*ku/lambda^2*(R12*lambda - R32*alpha);
    J_lmk(1,3) = f*ku/lambda^2*(R13*lambda - R33*alpha);
    J_lmk(2,1) = f*kv/lambda^2*(R21*lambda - R31*beta);
    J_lmk(2,2) = f*kv/lambda^2*(R22*lambda - R32*beta);
    J_lmk(2,3) = f*kv/lambda^2*(R23*lambda - R33*beta);
end
