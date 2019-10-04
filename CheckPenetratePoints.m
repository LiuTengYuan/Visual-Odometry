function XYZ_new = CheckPenetratePoints(currPos, XYZ, XYZ_L, XYZ_R)

XYZ_new = XYZ;
pL = [XYZ_L(1:3,1)';XYZ_L(1:3,2)';XYZ_L(1:3,3)'];
nL = cross(pL(1,:)-pL(2,:), pL(1,:)-pL(3,:));
pR = [XYZ_R(1:3,1)';XYZ_R(1:3,2)';XYZ_R(1:3,3)'];
nR = cross(pR(1,:)-pR(2,:), pR(1,:)-pR(3,:));
direction = (XYZ_L(1:2,1)-XYZ_L(1:2,2))==0; %[1 0]->x const; [0 1]->y const
if direction(1)
    XYZLBuilding = XYZ_L(:,1:find(XYZ_L(1,:)~=XYZ_L(1,1),1)-1);
    XYZRBuilding = XYZ_R(:,1:find(XYZ_R(1,:)~=XYZ_R(1,1),1)-1);
    segmentLmin = min(XYZLBuilding(2,:)); segmentLmax = max(XYZLBuilding(2,:));
    segmentRmin = min(XYZRBuilding(2,:)); segmentRmax = max(XYZRBuilding(2,:));
else
    XYZLBuilding = XYZ_L(:,1:find(XYZ_L(2,:)~=XYZ_L(2,1),1)-1);
    XYZRBuilding = XYZ_R(:,1:find(XYZ_R(2,:)~=XYZ_R(2,1),1)-1);
    segmentLmin = min(XYZLBuilding(1,:)); segmentLmax = max(XYZLBuilding(1,:));
    segmentRmin = min(XYZRBuilding(1,:)); segmentRmax = max(XYZRBuilding(1,:));
end

% %Draw Plane (testing)
% grid on; hold all;
% plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'k.')
% plot3(XYZ_L(1,:),XYZ_L(2,:),XYZ_L(3,:),'b.')
% plot3(XYZ_R(1,:),XYZ_R(2,:),XYZ_R(3,:),'c.')
% patch(pL(:,1),pL(:,2),pL(:,3),'r');
% patch(pR(:,1),pR(:,2),pR(:,3),'r');
% DrawPlane(pL(1,:),nL);
% DrawPlane(pR(1,:),nR);

CheckPenetrate = zeros(length(XYZ),2);
CheckDelete = false(length(XYZ),1);
for i = 1:length(XYZ)
    targetPos = XYZ(1:3,i)';
    [IL,checkL]=plane_line_intersect(nL,pL(2,:),currPos,targetPos);
    [IR,checkR]=plane_line_intersect(nR,pR(2,:),currPos,targetPos);
    if IL == targetPos
        checkL = 0;
    end
    if IR == targetPos
        checkR = 0;
    end
    if direction(1)
        if checkL==1 && (IL(2) < segmentLmin || IL(2) > segmentLmax)
            checkL = 0;
        end
        if checkR==1 && (IR(2) < segmentRmin || IR(2) > segmentRmax)
            checkR = 0;
        end
    else
        if checkL==1 && (IL(1) < segmentLmin || IL(1) > segmentLmax)
            checkL = 0;
        end
        if checkR==1 && (IR(1) < segmentRmin || IR(1) > segmentRmax)
            checkR = 0;
        end
    end
    CheckPenetrate(i,:) = [checkL checkR];
    if CheckPenetrate(i,1) == 1 || CheckPenetrate(i,2) == 1
       CheckDelete(i) = true;
   end
end; clear i

XYZ_new(:,CheckDelete) = [];

% %Draw Penetration (testing)
% plot3(XYZ(1,:),XYZ(2,:),XYZ(3,:),'k.'); hold all;
% h = plot3(XYZ_new(1,:),XYZ_new(2,:),XYZ_new(3,:),'r.');
% hh = plot3(currPos(1),currPos(2),currPos(3),'r.','MarkerSize',20);
% for i = 1:length(XYZ_new)
%     hhh = plot3([currPos(1) XYZ_new(1,i)],[currPos(2) XYZ_new(2,i)],[currPos(3) XYZ_new(3,i)],'g');
% end; clear i
% delete(h); delete(hh); delete(hhh);

end