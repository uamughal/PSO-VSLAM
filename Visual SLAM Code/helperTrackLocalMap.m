function [refKeyFrameId, localKeyFrameIds, currPose, mapPointIdx, featureIdx] = ...
    helperTrackLocalMap(mapPoints, vSetKeyFrames, mapPointIdx, ...
    featureIdx, currPose, currFeatures, currPoints, intrinsics)
%helperTrackLocalMap Refine camera pose by tracking the local map
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.
%
%   Inputs
%   ------
%   mapPoints         - A helperMapPointSet objects storing map points
%   vSetKeyFrames     - An imageviewset storing key frames
%   mapPointsIndices  - Indices of map points observed in the current frame
%   featureIndices    - Indices of features in the current frame 
%                       corresponding to map points denoted by mapPointsIndices                      
%   currPose          - Current camera pose
%   currFeatures      - ORB Features in the current frame 
%   currPoints        - Feature points in the current frame
%   intrinsics        - Camera intrinsics 
%   
%   Outputs
%   -------
%   mapPoints         - A helperMapPointSet objects storing map points
%   localKeyFrameIds  - ViewIds of the local key frames 
%   currPose          - Refined camera pose of the current frame
%   mapPointIdx       - Indices of map points observed in the current frame
%   featureIdx        - Indices of features in the current frame corresponding
%                       to mapPointIdx   

%   Copyright 2019 The MathWorks, Inc.

scaleFactor = 1.2;
numLevels   = 8;
imageSize   = [480, 640];

[refKeyFrameId, locaPointsIndices, localKeyFrameIds] = ...
    updateRefKeyFrameAndLocalPoints(mapPoints, vSetKeyFrames, mapPointIdx);

% Project the map into the frame and search for more map point correspondences
newmapPointIdx = setdiff(locaPointsIndices, mapPointIdx,'stable');

localMapPoints = mapPoints.Locations(newmapPointIdx, :);
viewDirection  = mapPoints.ViewDirections(newmapPointIdx, :);

[localFeatures, localPoints] = getFeaturesAndPoints(vSetKeyFrames.Views, ...
    mapPoints, newmapPointIdx);

% Filter out outliers 
[inlierIndex, predictedScales, viewAngles] = removeOutlierMapPoints(mapPoints, ...
    currPose, intrinsics, newmapPointIdx, scaleFactor, numLevels, imageSize);

newmapPointIdx = newmapPointIdx(inlierIndex);
localMapPoints      = localMapPoints(inlierIndex,:);
localFeatures       = localFeatures(inlierIndex,:);
locaValidPoints     = localPoints(inlierIndex,:);

unmatchedfeatureIdx = setdiff(cast((1:size( currFeatures.Features, 1)).', 'uint32'), ...
    featureIdx,'stable');
unmatchedFeatures       = currFeatures.Features(unmatchedfeatureIdx, :);
unmatchedValidPoints    = currPoints(unmatchedfeatureIdx);

[R, t]          = cameraPoseToExtrinsics(currPose.Rotation, currPose.Translation);
projectedPoints = worldToImage(intrinsics, R, t, localMapPoints);

% Search radius depends on scale and view direction
searchRadius    = 4*ones(size(localFeatures, 1), 1);
searchRadius(viewAngles<3) = 2.5;
searchRadius    = searchRadius.*predictedScales;

indexPairs = helperMatchFeaturesInRadius(localFeatures, unmatchedFeatures, ...
    locaValidPoints, unmatchedValidPoints, projectedPoints, searchRadius, ...
    max(1, predictedScales/scaleFactor), predictedScales);

% Refine camera pose with more 3D-to-2D correspondences
mapPointIdx   = [newmapPointIdx(indexPairs(:,1)); mapPointIdx];
featureIdx     = [unmatchedfeatureIdx(indexPairs(:,2)); featureIdx];
matchedMapPoints   = mapPoints.Locations(mapPointIdx,:);
matchedImagePoints = currPoints.Location(featureIdx,:);

% Refine camera pose only
currPose = bundleAdjustmentMotion(matchedMapPoints, matchedImagePoints, ...
    currPose, intrinsics, 'PointsUndistorted', true, ...
    'AbsoluteTolerance', 1e-7, 'RelativeTolerance', 1e-15,'MaxIteration', 20);
end

function [refKeyFrameId, locaPointsIndices, localKeyFrameIds] = ...
    updateRefKeyFrameAndLocalPoints(mapPoints, vSetKeyFrames, pointIndices)

% Get key frames K1 that observe map points in the current key frame
K1IDs = vertcat(mapPoints.Observations{pointIndices, 1});

% The reference key frame has the most covisible map points 
refKeyFrameId = mode(K1IDs);

% Retrieve key frames K2 that are connected to K1
K1IDs = unique(K1IDs);
localKeyFrameIds = K1IDs;
for i = 1:numel(K1IDs)
    viewTables = connectedViews(vSetKeyFrames, K1IDs(i));
    if isempty(intersect(viewTables.ViewId, localKeyFrameIds))
        localKeyFrameIds = [localKeyFrameIds; viewTables.ViewId]; %#ok<AGROW>
    end
end

locaPointsIndices = sort(getMapPointIndex(mapPoints, localKeyFrameIds));
locaPointsIndices = locaPointsIndices(mapPoints.Validity(locaPointsIndices));
end

function [features, validPoints] = getFeaturesAndPoints(views, mapPoints, mapPointIdx)

% Efficiently retrieve features and image points corresponding to map points
% denoted by mapPointIdx
allIndices = zeros(1, numel(mapPointIdx));
viewIds    = zeros(1, numel(mapPointIdx));

% ViewId and offset pair
count = []; % (ViewId, NumFeatures)
observations  = mapPoints.Observations;
viewsFeatures = views.Features;
viewsPoints   = views.Points;

for i = 1:numel(mapPointIdx)
    index3d       = mapPointIdx(i);
    observation   = observations(index3d, :);
    distIndex     = observation{5};
    viewId        = observation{1}(distIndex);
    viewIds(i)    = viewId;
    
    if isempty(count)
        count = [viewId, size(viewsFeatures{viewId},1)];
    elseif ~any(count(:,1) == viewIds(i))
        count = [count; viewId, size(viewsFeatures{viewId},1)];
    end
    
    idx = find(count(:,1)==viewId);
    
    if idx > 1
        offset = sum(count(1:idx-1,2));
    else
        offset = 0;
    end
    allIndices(i) = observation{2}(distIndex) + offset;
end

uIds = count(:,1);

% Concatenating features and points and indexing once is faster than
% accessing via a for loop
allFeatures = vertcat(viewsFeatures{uIds});
allPoints   = vertcat(viewsPoints{uIds});
features    = allFeatures(allIndices, :);
validPoints = allPoints(allIndices);
end

function [inliers, predictedScales, viewAngles] = removeOutlierMapPoints(...
    mapPoints, pose, intrinsics, localPointsIndices, scaleFactor, ...
    numLevels, imageSize)

% 1) Points within the image bounds
xyzPoints = mapPoints.Locations(localPointsIndices, :);
isInImage = helperFindProjectedPointsInImage(xyzPoints, pose, intrinsics, imageSize);

% 2) Parallax less than 40 degrees
cameraNormVector = [0 0 1] * pose.Rotation;
cameraToPoints   = xyzPoints - pose.Translation;
viewDirection    = mapPoints.ViewDirections(localPointsIndices, :);
validByView      = sum(viewDirection.*cameraToPoints, 2) > ...
    cosd(40)*(vecnorm(cameraToPoints, 2, 2));

% 3) Distance from map point to camera center is in the range of scale
% invariant depth
minDist          = mapPoints.MinDistance(localPointsIndices);
maxDist          = mapPoints.MaxDistance(localPointsIndices);
dist             = vecnorm(xyzPoints - pose.Translation, 2, 2);

minScaleInvariantDist = 0.8*minDist;
maxScaleInvariantDist = 1.2*maxDist;

validByDistance  = dist > minScaleInvariantDist & dist < maxScaleInvariantDist;

inliers          = isInImage & validByView & validByDistance;

% Predicted scales
level= ceil(log(maxDist ./ dist)./log(scaleFactor));
level(level<0)   = 0;
level(level>=numLevels-1) = numLevels-1;
predictedScales  = scaleFactor.^level;

% View angles
viewAngles       = acosd(sum(cameraNormVector.*cameraToPoints, 2) ./ ...
    vecnorm(cameraToPoints, 2, 2));

predictedScales  = predictedScales(inliers);
viewAngles       = viewAngles(inliers);

end
