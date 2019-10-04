%% Draw a line between optical center and 3Dpoints
function DrawFisheyeProjectionLines(O,M,P,color)
    if nargin <= 3
        color = [0.5,0.5,0.5];
    end
% to explain the projection of points into the image plane for the
% perspective
[r,c]=size(P);
for i=1:c
    plot3([O(1) M(1,i)],[O(2) M(2,i)],[O(3) M(3,i)],'Color',color)
    plot3([M(1,i) P(1,i)],[M(2,i) P(2,i)],[M(3,i) P(3,i)],'Color',color)
end
