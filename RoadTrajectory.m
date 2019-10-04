function [x0_v,y0_v,z0_v,a_v,b_v,c_v,MODE,TrueDistance] = RoadTrajectory(FrameRate,VS,segment,sec,curveRadius,Npose)

VSms = VS*1000/3600; %(m/s)
Nsegmentpose = FrameRate*sec; %(pose) Number of poses per segment
Distancesegment = VSms*sec; %(m) Distance per segment
Distancepose = VSms/FrameRate; %(m) Distance per pose
TurnSec = curveRadius/VSms; %(s) Second for turning
TurnDistance = curveRadius*(1-cos(pi/4)); %(m) Distance for turning
mode_before = 0; MODE = zeros(segment,1);

x0_v = zeros(1,Npose);
y0_v = zeros(1,Npose);
c_v = zeros(1,Npose); %yaw

%create different mode for each segment; modes include (up,down,left,right)
for i = 1:segment
    %for VO, the first x,y,z,raw,pitch,yaw must be zero
    if i == 1
        MODE(1) = 1;
        mode_before = 1;
        continue
    end
    while 1
        mode = randi([-2 2]);
        if mode~=0
            break
        end
    end
    while mode == -mode_before
        while 1
            mode = randi([-2 2]);
            if mode~=0
                break
            end
        end
    end
    MODE(i) = mode;
    mode_before = mode;
end; clear i

for i = 1:segment
    begin = (i-1)*Nsegmentpose+1;
    finish = i*Nsegmentpose;
    
    if begin > 1
        initialx = x0_v(begin-1);
        initialy = y0_v(begin-1);
        initialc = c_v(begin-1);
    else
        initialx = 0;
        initialy = 0;
        initialc = 0;
        %         if MODE(i) == -1
        %             initialc = pi; %or -pi
        %         end
    end
    
    if begin > 1 && MODE(i)~=MODE(i-1)
        mode_before = MODE(i-1);
    else
        mode_before = 0;
    end
    if finish < Npose && MODE(i)~=MODE(i+1)
        mode_after = MODE(i+1);
    else
        mode_after = 0;
    end
    
    %%
    switch MODE(i)
        case 1 %(+1,0)
            finalyaw = 0;
            yaw_before = linspace(initialc,finalyaw,round(FrameRate*TurnSec/2)+1+1);
            yaw_before(1) = [];
            yaw = [yaw_before finalyaw*ones(1,(Nsegmentpose-length(yaw_before)))];
            if mode_after
                if mode_after == 2 %(0,+1)
                    predictedyaw = (pi/2-finalyaw)/2+finalyaw;
                else %(0,-1)
                    predictedyaw = (-pi/2-finalyaw)/2+finalyaw;
                end
            else
                predictedyaw = finalyaw;
            end
            yaw_predicted = linspace(finalyaw,predictedyaw,round(FrameRate*TurnSec/2)+1);
            yaw(end-length(yaw_predicted)+1:end) = yaw_predicted;
            
            if begin>1
                xarray = linspace(initialx,initialx+Distancesegment,Nsegmentpose+1);
                xarray(1) = [];
            else
                xarray = linspace(initialx,initialx+Distancesegment,Nsegmentpose);
            end
            yarray = initialy*ones(1,Nsegmentpose);
            if mode_before
                if mode_before == 2 %(0,+1)
                    mode = 2; %(90->0)
                else %(0,-1)
                    mode = 1; %(-90->0)
                end
                [XRbefore,YRbefore] = ModeBefore(initialx,initialy,initialc,curveRadius,yaw_before,yaw,mode);
                xx = linspace(XRbefore(end),initialx+Distancesegment-TurnDistance,Nsegmentpose-length(XRbefore)+1);
                xx(1) = [];
                xarray = [XRbefore xx];
                yarray = [YRbefore YRbefore(end)*ones(1,Nsegmentpose-length(YRbefore))];
            end
            if mode_after
                if mode_after == 2 %(0,+1)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))+(Distancesegment-curveRadius-TurnDistance);
                        finalpointy = yarray(length(yaw_before))+TurnDistance;
                    else
                        finalpointx = initialx+(Distancesegment-TurnDistance);
                        finalpointy = initialy+TurnDistance;
                    end
                    mode = 1; %(0->90)
                else %(0,-1)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))+(Distancesegment-curveRadius-TurnDistance);
                        finalpointy = yarray(length(yaw_before))-TurnDistance;
                    else
                        finalpointx = initialx+(Distancesegment-TurnDistance);
                        finalpointy = initialy-TurnDistance;
                    end
                    mode = 2; %(0->-90)
                end
                [XRafter,YRafter] = ModeAfter(finalpointx,finalpointy,...
                    predictedyaw,curveRadius,yaw_predicted,yaw,mode);
                xx = linspace(xarray(length(XRafter)+1),XRafter(1),Nsegmentpose-2*length(XRafter)+1);
                xx(end) = [];
                xarray(length(XRafter)+1:end-length(XRafter)) = xx;
                xarray(end-length(XRafter)+1:end) = XRafter;
                yarray(end-length(YRafter)+1:end) = YRafter;
            end
            
            x0_v(begin:finish) = xarray;
            y0_v(begin:finish) = yarray;
            c_v(begin:finish) = yaw;
        case -1 %(-1,0)
            if abs(initialc-(pi-pi/2/2)) < 1e-1
                finalyaw = pi;
            elseif abs(initialc+(pi-pi/2/2)) < 1e-1
                finalyaw = -pi;
            else
                finalyaw = initialc;
            end
            yaw_before = linspace(initialc,finalyaw,round(FrameRate*TurnSec/2)+1+1);
            yaw_before(1) = [];
            yaw = [yaw_before finalyaw*ones(1,(Nsegmentpose-length(yaw_before)))];
            if mode_after
                if mode_after == 2 %(0,+1)
                    finalyawinverse = pi;
                    predictedyaw = (pi/2-finalyawinverse)/2+finalyawinverse;
                else %(0,-1)
                    finalyawinverse = -pi;
                    predictedyaw = (-pi/2-finalyawinverse)/2+finalyawinverse;
                end
            else
                finalyawinverse = finalyaw;
                predictedyaw = finalyaw;
            end
            yaw_predicted = linspace(finalyawinverse,predictedyaw,round(FrameRate*TurnSec/2)+1);
            yaw(end-length(yaw_predicted)+1:end) = yaw_predicted;
            
            xarray = linspace(initialx,initialx-Distancesegment,Nsegmentpose+1);
            xarray(1) = [];
            yarray = initialy*ones(1,Nsegmentpose);
            if mode_before
                if mode_before == 2 %(0,+1)
                    mode = 1; %(90->180)
                else %(0,-1)
                    mode = 2; %(-90->-180)
                end
                [XRbefore,YRbefore] = ModeBefore(initialx,initialy,initialc,curveRadius,yaw_before,yaw,mode);
                xx = linspace(XRbefore(end),initialx-Distancesegment+TurnDistance,Nsegmentpose-length(XRbefore)+1);
                xx(1) = [];
                xarray = [XRbefore xx];
                yarray = [YRbefore YRbefore(end)*ones(1,Nsegmentpose-length(YRbefore))];
            end
            if mode_after
                if mode_after == 2 %(0,+1)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))-(Distancesegment-curveRadius-TurnDistance);
                        finalpointy = yarray(length(yaw_before))+TurnDistance;
                    else
                        finalpointx = initialx-(Distancesegment-TurnDistance);
                        finalpointy = initialy+TurnDistance;
                    end
                    mode = 2; %(180->90)
                else %(0,-1)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))-(Distancesegment-curveRadius-TurnDistance);
                        finalpointy = yarray(length(yaw_before))-TurnDistance;
                    else
                        finalpointx = initialx-(Distancesegment-TurnDistance);
                        finalpointy = initialy-TurnDistance;
                    end
                    mode = 1; %(-180->-90)
                end
                [XRafter,YRafter] = ModeAfter(finalpointx,finalpointy,...
                    predictedyaw,curveRadius,yaw_predicted,yaw,mode);
                xx = linspace(xarray(length(XRafter)+1),XRafter(1),Nsegmentpose-2*length(XRafter)+1);
                xx(end) = [];
                xarray(length(XRafter)+1:end-length(XRafter)) = xx;
                xarray(end-length(XRafter)+1:end) = XRafter;
                yarray(end-length(YRafter)+1:end) = YRafter;
            end
            
            x0_v(begin:finish) = xarray;
            y0_v(begin:finish) = yarray;
            c_v(begin:finish) = yaw;
        case 2 %(0,+1)
            finalyaw = pi/2;
            yaw_before = linspace(initialc,finalyaw,round(FrameRate*TurnSec/2)+1+1);
            yaw_before(1) = [];
            yaw = [yaw_before finalyaw*ones(1,(Nsegmentpose-length(yaw_before)))];
            if mode_after
                if mode_after == 1 %(+1,0)
                    predictedyaw = (0-finalyaw)/2+finalyaw;
                else %(-1,0)
                    predictedyaw = (pi-finalyaw)/2+finalyaw;
                end
            else
                predictedyaw = finalyaw;
            end
            yaw_predicted = linspace(finalyaw,predictedyaw,round(FrameRate*TurnSec/2)+1);
            yaw(end-length(yaw_predicted)+1:end) = yaw_predicted;
            
            yarray = linspace(initialy,initialy+Distancesegment,Nsegmentpose+1);
            yarray(1) = [];
            xarray = initialx*ones(1,Nsegmentpose);
            if mode_before
                if mode_before == 1 %(+1,0)
                    mode = 1; %(0->90)
                else %(-1,0)
                    mode = 2; %(180->90)
                end
                [XRbefore,YRbefore] = ModeBefore(initialx,initialy,initialc,curveRadius,yaw_before,yaw,mode);
                xarray = [XRbefore XRbefore(end)*ones(1,Nsegmentpose-length(XRbefore))];
                yy = linspace(YRbefore(end),initialy+Distancesegment-TurnDistance,Nsegmentpose-length(YRbefore)+1);
                yy(1) = [];
                yarray = [YRbefore yy];
            end
            if mode_after
                if mode_after == 1 %(+1,0)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))+TurnDistance;
                        finalpointy = yarray(length(yaw_before))+(Distancesegment-curveRadius-TurnDistance);
                    else
                        finalpointx = initialx+TurnDistance;
                        finalpointy = initialy+(Distancesegment-TurnDistance);
                    end
                    mode = 2; %(90->0)
                else %(-1,0)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))-TurnDistance;
                        finalpointy = yarray(length(yaw_before))+(Distancesegment-curveRadius-TurnDistance);
                    else
                        finalpointx = initialx-TurnDistance;
                        finalpointy = initialy+(Distancesegment-TurnDistance);
                    end
                    mode = 1; %(90->180)
                end
                [XRafter,YRafter] = ModeAfter(finalpointx,finalpointy,...
                    predictedyaw,curveRadius,yaw_predicted,yaw,mode);
                yy = linspace(yarray(length(YRafter)+1),YRafter(1),Nsegmentpose-2*length(YRafter)+1);
                yy(end) = [];
                yarray(length(YRafter)+1:end-length(YRafter)) = yy;
                xarray(end-length(XRafter)+1:end) = XRafter;
                yarray(end-length(YRafter)+1:end) = YRafter;
            end
            
            x0_v(begin:finish) = xarray;
            y0_v(begin:finish) = yarray;
            c_v(begin:finish) = yaw;
        case -2 %(0,-1)
            finalyaw = -pi/2;
            yaw_before = linspace(initialc,finalyaw,round(FrameRate*TurnSec/2)+1+1);
            yaw_before(1) = [];
            yaw = [yaw_before finalyaw*ones(1,(Nsegmentpose-length(yaw_before)))];
            if mode_after
                if mode_after == 1 %(+1,0)
                    predictedyaw = (0-finalyaw)/2+finalyaw;
                else %(-1,0)
                    predictedyaw = (-pi-finalyaw)/2+finalyaw;
                end
            else
                predictedyaw = finalyaw;
            end
            yaw_predicted = linspace(finalyaw,predictedyaw,round(FrameRate*TurnSec/2)+1);
            yaw(end-length(yaw_predicted)+1:end) = yaw_predicted;
            
            yarray = linspace(initialy,initialy-Distancesegment,Nsegmentpose+1);
            yarray(1) = [];
            xarray = initialx*ones(1,Nsegmentpose);
            if mode_before
                if mode_before == 1 %(+1,0)
                    mode = 2; %(0->-90)
                else %(-1,0)
                    mode = 1; %(-180->-90)
                end
                [XRbefore,YRbefore] = ModeBefore(initialx,initialy,initialc,curveRadius,yaw_before,yaw,mode);
                xarray = [XRbefore XRbefore(end)*ones(1,Nsegmentpose-length(XRbefore))];
                yy = linspace(YRbefore(end),initialy-Distancesegment+TurnDistance,Nsegmentpose-length(YRbefore)+1);
                yy(1) = [];
                yarray = [YRbefore yy];
            end
            if mode_after
                if mode_after == 1 %(+1,0)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))+TurnDistance;
                        finalpointy = yarray(length(yaw_before))-(Distancesegment-curveRadius-TurnDistance);
                    else
                        finalpointx = initialx+TurnDistance;
                        finalpointy = initialy-(Distancesegment-TurnDistance);
                    end
                    mode = 1; %(-90->0)
                else %(-1,0)
                    if mode_before
                        finalpointx = xarray(length(yaw_before))-TurnDistance;
                        finalpointy = yarray(length(yaw_before))-(Distancesegment-curveRadius-TurnDistance);
                    else
                        finalpointx = initialx-TurnDistance;
                        finalpointy = initialy-(Distancesegment-TurnDistance);
                    end
                    mode = 2; %(-90->-180)
                end
                [XRafter,YRafter] = ModeAfter(finalpointx,finalpointy,...
                    predictedyaw,curveRadius,yaw_predicted,yaw,mode);
                yy = linspace(yarray(length(YRafter)+1),YRafter(1),Nsegmentpose-2*length(YRafter)+1);
                yy(end) = [];
                yarray(length(YRafter)+1:end-length(YRafter)) = yy;
                xarray(end-length(XRafter)+1:end) = XRafter;
                yarray(end-length(YRafter)+1:end) = YRafter;
            end
            
            x0_v(begin:finish) = xarray;
            y0_v(begin:finish) = yarray;
            c_v(begin:finish) = yaw;
    end
end; clear i

z0_v = zeros(1,Npose);
a_v = zeros(1,Npose);
b_v = zeros(1,Npose);

TrueDistance = zeros(1,Npose);
for i = 2:Npose
    TrueDistance(i) = TrueDistance(i-1)+norm([x0_v(i)-x0_v(i-1) y0_v(i)-y0_v(i-1) z0_v(i)-z0_v(i-1)]);
end; clear i


%--------------------------------------------------------------------------
    function [XR,YR] = ModeBefore(initialx,initialy,initialc,curveRadius,yaw_before,yaw,mode)
        if mode == 1
            %mode1:(0->90)(90->180)(-90->0)(-180->-90)
            xr = initialx-curveRadius*sin(initialc);
            yr = initialy+curveRadius*cos(initialc);
            XR = zeros(1,length(yaw_before)); YR = zeros(1,length(yaw_before));
            for theda = 1:length(yaw_before)
                XR(theda) = xr+curveRadius*sin(yaw(theda));
                YR(theda) = yr-curveRadius*cos(yaw(theda));
            end
        else
            %mode2:(0->-90)(90->0)(-90->-180)(180->90)
            xr = initialx+curveRadius*sin(initialc);
            yr = initialy-curveRadius*cos(initialc);
            XR = zeros(1,length(yaw_before)); YR = zeros(1,length(yaw_before));
            for theda = 1:length(yaw_before)
                XR(theda) = xr-curveRadius*sin(yaw(theda));
                YR(theda) = yr+curveRadius*cos(yaw(theda));
            end
        end
    end
%--------------------------------------------------------------------------
    function [XR,YR] = ModeAfter(finalpointx,finalpointy,predictedyaw,curveRadius,yaw_predicted,yaw,mode)
        if mode == 1
            %mode1:(0->90)(90->180)(-90->0)(-180->-90)
            xr = finalpointx-curveRadius*sin(predictedyaw);
            yr = finalpointy+curveRadius*cos(predictedyaw);
            XR = zeros(1,length(yaw_predicted)); YR = zeros(1,length(yaw_predicted));
            for theda = length(yaw)-length(yaw_predicted)+1:length(yaw)
                index = theda-(length(yaw)-length(yaw_predicted));
                XR(index) = xr+curveRadius*sin(yaw(theda));
                YR(index) = yr-curveRadius*cos(yaw(theda));
            end
        else
            %mode2:(0->-90)(90->0)(-90->-180)(180->90)
            xr = finalpointx+curveRadius*sin(predictedyaw);
            yr = finalpointy-curveRadius*cos(predictedyaw);
            XR = zeros(1,length(yaw_predicted)); YR = zeros(1,length(yaw_predicted));
            for theda = length(yaw)-length(yaw_predicted)+1:length(yaw)
                index = theda-(length(yaw)-length(yaw_predicted));
                XR(index) = xr-curveRadius*sin(yaw(theda));
                YR(index) = yr+curveRadius*cos(yaw(theda));
            end
        end
    end


end
