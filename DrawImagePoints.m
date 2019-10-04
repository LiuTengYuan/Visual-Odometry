%% Draw a set on Image points
function DrawImagePoints(P,Nu,Nv,color,point)
    if nargin <= 3
        color = [0,0,0];
    end
    if nargin <= 4
        point = '*';
    end
    %draws points on the image plane  (homogeneous cordinates)
    [r,c]=size(P);
    % Normalize by third homogeneous coordinate
    for column=1:c
        P(1:r-1,column) = P(1:r-1,column)/P(3,column);
    end
    % Plot
    plot(P(1,1:c),P(2,1:c),point,'Color',color);
    % Restrict to image size (Nu x Nv) pixels
    axis([0 Nu 0 Nv]);
    daspect([1 1 1])
    set(gca,'YDir','reverse');
    
    xlabel('u','FontSize',14);
    ylabel('v','FontSize',14);
end