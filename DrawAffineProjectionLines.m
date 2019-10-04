%% Draw a line between image plane and 3D points
function DrawAffineProjectionLines(z,P,colour)
% to explain the projection of points into the image plane for the affine
[r,c]=size(P);
for column=1:c
    plot3([P(1,column) P(1,column)], [P(2,column) P(2,column)], [z(3) P(3,column)],'color',colour);   %optic center
end

end