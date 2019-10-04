function Line = CreateRoadLine(side,width,MODE,FrameRate,VS,segment,sec,curveRadius,Npose)
%side=1(right side); side=-1(left side); side=0(center)
x = zeros(1,segment+1);
y = zeros(1,segment+1);
z = zeros(1,segment+1);
VSms = VS*1000/3600; %(m/s)
Distancesegment = VSms*sec; %(m) Distance per segment
length = side*width;
y(1) = -side*width;
for i = 1:segment
    initialx = x(i);
    initialy = y(i);
    if i > 1 && MODE(i)~=MODE(i-1)
        mode_before = MODE(i-1);
    else
        mode_before = 0;
    end
    if i < segment && MODE(i)~=MODE(i+1)
        mode_after = MODE(i+1);
    else
        mode_after = 0;
    end
    switch MODE(i)
        case 1
            if ~mode_before
                turn = 0;
            else
                if mode_before == 2
                    turn = 1;
                else %mode_before == -2
                    turn = -1;
                end
            end
            if ~mode_after
                turn = turn+0;
            else
                if mode_after == 2
                    turn = turn-1;
                else %mode_after == -2
                    turn = turn+1;
                end
            end
            x(i+1) = initialx+Distancesegment-length*turn;
            y(i+1) = initialy;
        case -1
            if ~mode_before
                turn = 0;
            else
                if mode_before == 2
                    turn = -1;
                else %mode_before == -2
                    turn = 1;
                end
            end
            if ~mode_after
                turn = turn+0;
            else
                if mode_after == 2
                    turn = turn+1;
                else %mode_after == -2
                    turn = turn-1;
                end
            end
            x(i+1) = initialx-Distancesegment+length*turn;
            y(i+1) = initialy;
        case 2
            if ~mode_before
                turn = 0;
            else
                if mode_before == 1
                    turn = -1;
                else %mode_before == -1
                    turn = +1;
                end
            end
            if ~mode_after
                turn = turn+0;
            else
                if mode_after == 1
                    turn = turn+1;
                else %mode_after == -1
                    turn = turn-1;
                end
            end
            x(i+1) = initialx;
            y(i+1) = initialy+Distancesegment-length*turn;
        case -2
            if ~mode_before
                turn = 0;
            else
                if mode_before == 1
                    turn = 1;
                else %mode_before == -1
                    turn = -1;
                end
            end
            if ~mode_after
                turn = turn+0;
            else
                if mode_after == 1
                    turn = turn-1;
                else %mode_after == -1
                    turn = turn+1;
                end
            end
            x(i+1) = initialx;
            y(i+1) = initialy-Distancesegment+length*turn;
    end
end
Line = [x;y;z];
end