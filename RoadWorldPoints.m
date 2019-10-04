function [XYZ,XYZ_segment,XYZ_L,XYZ_R] = RoadWorldPoints...
    (x0_v,y0_v,z0_v,MODE,TrueDistance,FrameRate,VS,segment,sec,curveRadius,Npose)
XYZ = [];
XYZ_segment = cell(segment,1);
XYZ_L = cell(segment,1);
XYZ_R = cell(segment,1);
%% Road Lines
roadwidth = 3.5;
sidewalkwidth = 1.5;
carline = CreateRoadLine(0,0,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
inv_carline = CreateRoadLine(-1,roadwidth,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
centerline = CreateRoadLine(-1,roadwidth/2,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
Lroadline = CreateRoadLine(-1,roadwidth*3/2,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
Rroadline = CreateRoadLine(1,roadwidth/2,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
Lsidewalk = CreateRoadLine(-1,roadwidth*3/2+sidewalkwidth,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);
Rsidewalk = CreateRoadLine(1,roadwidth/2+sidewalkwidth,MODE,FrameRate,VS,segment,sec,curveRadius,Npose);

% %Plot Road Scenario (testing)
% hold all
% plot3(carline(1,:),carline(2,:),carline(3,:),'c:','LineWidth',2)
% plot3(inv_carline(1,:),inv_carline(2,:),inv_carline(3,:),'c:','LineWidth',2)
% plot3(centerline(1,:),centerline(2,:),centerline(3,:),'k--','LineWidth',2)
% plot3(Lroadline(1,:),Lroadline(2,:),Lroadline(3,:),'k-','LineWidth',2)
% plot3(Rroadline(1,:),Rroadline(2,:),Rroadline(3,:),'k-','LineWidth',2)
% plot3(Lsidewalk(1,:),Lsidewalk(2,:),Lsidewalk(3,:),'k-','LineWidth',2)
% plot3(Rsidewalk(1,:),Rsidewalk(2,:),Rsidewalk(3,:),'k-','LineWidth',2)
% plot3(x0_v(:),y0_v(:),z0_v(:),'b.-','MarkerSize',10);
% plot3(x0_v(Npose/2),y0_v(Npose/2),z0_v(Npose/2),'r.','MarkerSize',15);
% str = {['Distance: ' num2str(TrueDistance(Npose/2)) ' m'],['Velocity: ' num2str(VS) ' km/hr']};
% text(x0_v(Npose/2),y0_v(Npose/2),z0_v(Npose/2),str,'FontWeight','bold','Color','r');
% grid on; daspect([1 1 1]); view([0,90])
% xlabel('X','FontSize',14)
% ylabel('Y','Fontsize',14)
% zlabel('Z','Fontsize',14)

%% Buildings for different floors
XYZ_LBuilding = CreateBuildingPoints(Lsidewalk);
XYZ_RBuilding = CreateBuildingPoints(Rsidewalk);

% %Plot Building Points (testing)
% for i = 1:segment
%     plot3(XYZ_LBuilding{i}(:,1),XYZ_LBuilding{i}(:,2),XYZ_LBuilding{i}(:,3),'b.')%,'Color',[0.5,0.5,0.5])
%     plot3(XYZ_RBuilding{i}(:,1),XYZ_RBuilding{i}(:,2),XYZ_RBuilding{i}(:,3),'r.')%,'Color',[0.5,0.5,0.5])
% end; clear i
% view([-135,70])

%% traffic light,street tree/lamp,utility pole

XYZ_LOthers = CreateOtherPoints(Lroadline);
XYZ_ROthers = CreateOtherPoints(Rroadline);

% %Plot Other Points (testing)
% for i = 1:segment
%     plot3(XYZ_LOthers{i}(:,1),XYZ_LOthers{i}(:,2),XYZ_LOthers{i}(:,3),'g.')
%     plot3(XYZ_ROthers{i}(:,1),XYZ_ROthers{i}(:,2),XYZ_ROthers{i}(:,3),'g.')
% end; clear i
% view([-135,70])

%% Transfer XYZ & add the forth term

% %Plot Trajectory (testing)
% filenameMovie = './results/Road Scenario';
% v = VideoWriter(filenameMovie);
% open(v); view([-180,90])
% for i = 1:Npose
%     if i~=1; delete(h); delete(t); end
%     h = plot3(x0_v(1:i),y0_v(1:i),z0_v(1:i),'y.-','MarkerSize',10);
%     str = {['Distance: ' num2str(TrueDistance(i)) ' m'],['Velocity: ' num2str(VS) ' km/hr']};
%     t = text(x0_v(i),y0_v(i),z0_v(i),str,'FontWeight','bold');
%     drawnow
%     %save and make the film
%     saveas(gcf,['./results/image_Scenario_' num2str(i,'%03d') '.fig']);
%     of = openfig(['./results/image_Scenario_' num2str(i,'%03d') '.fig']);
%     set(of,'renderer','zbuffer')
%     frame = getframe(of);
%     close(of);
%     delete(['./results/image_Scenario_' num2str(i,'%03d') '.fig']);
%     writeVideo(v,frame);
% end; clear i h t
% close(v);

for i = 1:segment
    XYZ_L(i) = {[XYZ_LBuilding{i};XYZ_LOthers{i}]};
    XYZ_L{i}(:,4) = 1; XYZ_L{i} = XYZ_L{i}';
    XYZ_R(i) = {[XYZ_RBuilding{i};XYZ_ROthers{i}]};
    XYZ_R{i}(:,4) = 1; XYZ_R{i} = XYZ_R{i}';
    XYZ_segment(i) = {[XYZ_L{i} XYZ_R{i}]};
    XYZ = [XYZ XYZ_segment{i}];
end; clear i


end