%% Draw a co-ordinate frame
%draws the three axes given the location of the origin 
function DrawWorldFrame(x0,x1,y0,y1,z0,z1)

    axis equal
    axis([x0,x1,y0,y1,z0,z1])

    xlabel('X','FontSize',14)
    ylabel('Y','Fontsize',14)
    zlabel('Z','Fontsize',14)

    grid on
    hold on
end