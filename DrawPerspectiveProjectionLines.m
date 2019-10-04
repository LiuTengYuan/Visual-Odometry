%% Draw a line between optical center and 3Dpoints
function DrawPerspectiveProjectionLines(O,P,color)
    if nargin <= 2
        color = [0.5,0.5,0.5];
    end
% to explain the projection of points into the image plane for the
% perspective
[r,c]=size(P);
for i=1:c
    plot3([O(1) P(1,i)],[O(2) P(2,i)],[O(3) P(3,i)],'Color',color)
end
