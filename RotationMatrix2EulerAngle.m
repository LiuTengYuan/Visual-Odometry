function [estimatedroll,estimatedpitch,estimatedyaw] = RotationMatrix2EulerAngle(RM)

%reference
%https://www.mathworks.com/help/robotics/ref/rotm2eul.html

 eul = rotm2eul(RM);
 estimatedroll = eul(3);
 estimatedpitch = eul(2);
 estimatedyaw = eul(1);
 
%reference
%http://www.gregslabaugh.net/publications/euler.pdf

% choisir = 0;
% if RM(3,1)~=1 && RM(3,1)~=-1
%     pitch1 = -asin(RM(3,1));
%     pitch2 = pi-pitch1;
%     roll1 = atan2(RM(3,2)/cos(pitch1),RM(3,3)/cos(pitch1));
%     roll2 = atan2(RM(3,2)/cos(pitch2),RM(3,3)/cos(pitch2));
%     yaw1 = atan2(RM(2,1)/cos(pitch1),RM(1,1)/cos(pitch1));
%     yaw2 = atan2(RM(2,1)/cos(pitch2),RM(1,1)/cos(pitch2));
%     choisir = 1;
% else
%     yaw = 0; %could be anything;set to zero
%     if RM(3,1)==-1
%         pitch = pi/2;
%         roll = yaw+atans(RM(1,2),RM(1,3));
%     else
%         pitch = -pi/2;
%         roll = -yaw+atan2(-RM(1,2),-RM(1,3));
%     end
% end
% 
% if choisir
%     if abs(pitch1-a_v) < abs(pitch2-a_v)
%         estimatedroll = roll1;
%         estimatedpitch = pitch1;
%         estimatedyaw = yaw1;
%     else
%         estimatedroll = roll2;
%         estimatedpitch = pitch2;
%         estimatedyaw = yaw2;
%     end
% else
%     estimatedroll = roll;
%     estimatedpitch = pitch;
%     estimatedyaw = yaw;
% end

end