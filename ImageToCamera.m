%% Convert from homogeneous co-ordinates in pixels to distance co-ordinates
%% in the camera frame
function I2C=ImageToCamera(C)
I2C = zeros(4,3);

I2C(1,1) = 1/C(10);
I2C(2,2) = 1/C(11);
I2C(1,3) = -C(8)/C(10);
I2C(2,3) = -C(9)/C(11);
I2C(3,3) = C(7);
I2C(4,3) = 1;

% Ki(4,:) = 0;
% 
% % co-ordinate in distance units
% p=Ki*P;
% 
% %co-ordinates in the image plane 
% p(1,:)=p(1,:)./p(3,:);
% p(2,:)=p(2,:)./p(3,:);
% p(3,:)=p(3,:)./p(3,:);
% 
% %the third co-ordinate gives the depth 
% %the focal length C(7) defines depth
% p(3,:)=p(3,:).*C(7);
% 
% %include homogenous co-ordinates
% p(4,:)=1;
end




