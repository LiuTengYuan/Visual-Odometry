function [Diff_EstimatedActual,Diff_EstimatedActual_O,TranslationError,RotationError,Distance,DistanceTrue]...
    = Difference_Error(vSet,groundTruthPoses)

Npose = length(groundTruthPoses.ViewId(:));
Diff_EstimatedActual = NaN(Npose,1); %(m)
Diff_EstimatedActual_O = NaN(Npose,3); %(deg)
Distance = NaN(Npose,1); %(m)
DistanceTrue = NaN(Npose,1); %(m)
TranslationError = NaN(Npose,1); %(%)
RotationError = NaN(Npose,3); %(deg/m)
for viewId = 1:Npose
    Diff_EstimatedActual(viewId) = norm(vSet.Views.Location{viewId}-...
        groundTruthPoses.Location{viewId});
    %angle(roll,pitch,yaw) from orientation is oppisite to actual angle
    estimatedRM = vSet.Views.Orientation{viewId};
    [estimatedroll,estimatedpitch,estimatedyaw] = RotationMatrix2EulerAngle(estimatedRM);
    actualRM = groundTruthPoses.Orientation{viewId};
    [actualroll,actualpitch,actualyaw] = RotationMatrix2EulerAngle(actualRM);
    if abs(abs(estimatedroll-actualroll)-2*pi) < 5
        estimatedroll = -estimatedroll;
    end
    if abs(abs(estimatedpitch-actualpitch)-2*pi) < 5
        estimatedpitch = -estimatedpitch;
    end
    if abs(abs(estimatedyaw-actualyaw)-2*pi) < 5
        estimatedyaw = -estimatedyaw;
    end
    Diff_EstimatedActual_O(viewId,:) = rad2deg([(estimatedroll-actualroll);...
        (estimatedpitch-actualpitch);(estimatedyaw-actualyaw)]);
    if viewId == 1
        Distance(viewId) = 0;
        DistanceTrue(viewId) = 0;
        TranslationError(viewId) = 0;
        RotationError(viewId) = 0;
    else
        Distance(viewId) = Distance(viewId-1)+...
            norm(vSet.Views.Location{viewId}-vSet.Views.Location{viewId-1});
        DistanceTrue(viewId) = DistanceTrue(viewId-1)+...
            norm(groundTruthPoses.Location{viewId}-groundTruthPoses.Location{viewId-1});
        TranslationError(viewId) = Diff_EstimatedActual(viewId)/DistanceTrue(viewId);
        RotationError(viewId,:) = Diff_EstimatedActual_O(viewId,:)/DistanceTrue(viewId);
    end
end

end