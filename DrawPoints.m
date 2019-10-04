%% Draw a set of points
function Points = DrawPoints(P,color)

    %draws a set of points given its 3D world cordinates
    Points = plot3(P(1,:),P(2,:),P(3,:),'.','Color',color);
end