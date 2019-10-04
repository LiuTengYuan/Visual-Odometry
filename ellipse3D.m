function ellipse = ellipse3D(x,y,z,rx,ry,c,a)
th = 0:pi/50:2*pi;
x_circle = rx * cos(th) + x;
y_circle = ry * sin(th) + y;
z_circle(1:length(th)) = z;
ellipse = plot3(x_circle,y_circle,z_circle);
fill3(x_circle,y_circle,z_circle,c,'FaceAlpha',a)
end