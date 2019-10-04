function XYZBuilding = CreateBuildingPoints(sidewalk)

XYZBuilding = cell(length(sidewalk)-1,1);
Building = cell(length(sidewalk)-1,1);
for i = 1:length(sidewalk)-1
    roaddirection = (sidewalk(1:2,i+1)-sidewalk(1:2,i))';
    roadlength = max(abs(roaddirection));
    unit_roaddirection = roaddirection/roadlength;
    
    XYZBuildingSegment = [];
    BuildingSegment = []; %[width height location_i location_f]
    buildingwidth = randi([5 8]);
    buildingheight = 3*randi([2 6]);
    buildinglocation_i = sidewalk(1:2,i)';
    buildinglocation_f = buildinglocation_i+buildingwidth*unit_roaddirection;
    restlength = roadlength;
    while restlength > buildingwidth
        BuildingSegment = [BuildingSegment;buildingwidth buildingheight buildinglocation_i buildinglocation_f];
        Npoints = buildingwidth*buildingheight;
        %random points for one building
        XYZ  = [buildinglocation_i 0]+rand(Npoints,3).*[unit_roaddirection 1].*...
            [buildingwidth buildingwidth buildingheight];
        %points for bulding edge(up,down,left,right)
        width = linspace(0,buildingwidth,2*buildingwidth)';
        height = linspace(0,buildingheight,2*buildingheight)';
%         XYZ_edge_D = [buildinglocation_i 0]+kron(width,[unit_roaddirection 0]);
        XYZ_edge_U = [buildinglocation_i buildingheight]+kron(width,[unit_roaddirection 0]);
        XYZ_edge_L = [buildinglocation_i 0]+kron(height,[0 0 1]);
        XYZ_edge_R = [buildinglocation_f 0]+kron(height,[0 0 1]);
        XYZBuildingSegment = [XYZBuildingSegment; XYZ; XYZ_edge_U; XYZ_edge_L; XYZ_edge_R];
        restlength = restlength-buildingwidth-0.5;
        buildinglocation_i = buildinglocation_i+(buildingwidth+0.5)*unit_roaddirection;
        buildingwidth = randi([5 8]);
        buildingheight = 3*randi([2 6]);
        buildinglocation_f = buildinglocation_i+buildingwidth*unit_roaddirection;
    end
    if restlength > 1.5
        buildingwidth = floor(restlength-0.5);
        buildinglocation_f = buildinglocation_i+buildingwidth*unit_roaddirection;
        BuildingSegment = [BuildingSegment;buildingwidth buildingheight buildinglocation_i buildinglocation_f];
        Npoints = buildingwidth*buildingheight;
        %random points for one building
        XYZ  = [buildinglocation_i 0]+rand(Npoints,3).*[unit_roaddirection 1].*...
            [buildingwidth buildingwidth buildingheight];
        %points for bulding edge(up,down,left,right)
        width = linspace(0,buildingwidth,2*buildingwidth)';
        height = linspace(0,buildingheight,2*buildingheight)';
%         XYZ_edge_D = [buildinglocation_i 0]+kron(width,[unit_roaddirection 0]);
        XYZ_edge_U = [buildinglocation_i buildingheight]+kron(width,[unit_roaddirection 0]);
        XYZ_edge_L = [buildinglocation_i 0]+kron(height,[0 0 1]);
        XYZ_edge_R = [buildinglocation_f 0]+kron(height,[0 0 1]);
        XYZBuildingSegment = [XYZBuildingSegment; XYZ; XYZ_edge_U; XYZ_edge_L; XYZ_edge_R];
    end
    
    XYZBuilding(i) = {XYZBuildingSegment};
    Building(i) = {BuildingSegment};
end

end