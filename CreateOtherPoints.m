function XYZOthers = CreateOtherPoints(roadline)

XYZOthers = cell(length(roadline)-1,1);
Others = cell(length(roadline)-1,1);
for i = 1:length(roadline)-1
    roaddirection = (roadline(1:2,i+1)-roadline(1:2,i))';
    roadlength = max(abs(roaddirection));
    unit_roaddirection = roaddirection/roadlength;
    
    XYZOthersSegment = [];
    OthersSegment = []; %[width height location_i location_f]
    buildingwidth = rand+0.5;
    buildingheight = 3*randi([1 3]);
    buildinglocation_i = roadline(1:2,i)';
    buildinglocation_f = buildinglocation_i+buildingwidth*unit_roaddirection;
    restlength = roadlength;
    while restlength > buildingwidth
        OthersSegment = [OthersSegment;buildingwidth buildingheight buildinglocation_i buildinglocation_f];
        Npoints = ceil(buildingwidth)*buildingheight;
        %random points for one building
        XYZ  = [buildinglocation_i 0]+rand(Npoints,3).*[unit_roaddirection 1].*...
            [buildingwidth buildingwidth buildingheight];
        %points for bulding edge(up,down,left,right)
        width = linspace(0,buildingwidth,2*ceil(buildingwidth))';
        height = linspace(0,buildingheight,2*buildingheight)';
%         XYZ_edge_D = [buildinglocation_i 0]+kron(width,[unit_roaddirection 0]);
        XYZ_edge_U = [buildinglocation_i buildingheight]+kron(width,[unit_roaddirection 0]);
        XYZ_edge_L = [buildinglocation_i 0]+kron(height,[0 0 1]);
        XYZ_edge_R = [buildinglocation_f 0]+kron(height,[0 0 1]);
        XYZOthersSegment = [XYZOthersSegment; XYZ; XYZ_edge_U; XYZ_edge_L; XYZ_edge_R];
        spacing = randi([1 3]);
        restlength = restlength-buildingwidth-spacing;
        buildinglocation_i = buildinglocation_i+(buildingwidth+spacing)*unit_roaddirection;
        buildingwidth = rand+0.5;
        buildingheight = 3*randi([1 3]);
        buildinglocation_f = buildinglocation_i+buildingwidth*unit_roaddirection;
    end
    
    XYZOthers(i) = {XYZOthersSegment};
    Others(i) = {OthersSegment};
end


end