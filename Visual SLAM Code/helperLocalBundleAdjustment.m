function [mapPoints, vSetKeyFrames] = helperLocalBundleAdjustment(mapPoints, ...
    vSetKeyFrames, currKeyframeId, intrinsics)
%helperLocalBundleAdjustment refine the pose of the current key frame and 
%   the map of the surrrounding scene.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019 The MathWorks, Inc.

% Connected key frames of the current key frame
covisViews    = connectedViews(vSetKeyFrames, currKeyframeId);
covisViewsIds = covisViews.ViewId;

% Identify the fixed key frames that are connected to the connected
% key frames of the current key frame
fixedViewIds  = [];
for i = 1:numel(covisViewsIds)
    tempViews = connectedViews(vSetKeyFrames, covisViewsIds(i));
    tempId    = tempViews.ViewId;
    
    for j = 1:numel(tempId)
        if ~ismember(tempId(j), [fixedViewIds; currKeyframeId; covisViewsIds])
            fixedViewIds = [fixedViewIds; tempId(j)]; %#ok<AGROW>
        end
    end
end

% Always fix the first key frame
if ismember(1, covisViewsIds)
    fixedViewIds  = [fixedViewIds; 1];
end

refinedKeyFrameIds = [unique([fixedViewIds; covisViewsIds]); currKeyframeId];

% Indices of map points observed by local key frames
mapPointIdx  = getMapPointIndex(mapPoints, [covisViewsIds; currKeyframeId]);
mapPointIdx  = mapPointIdx(mapPoints.Validity(mapPointIdx));

% Find point tracks across the local key frames
numPoints    = numel(mapPointIdx);
tracks       = repmat(pointTrack(0,[0 0]),1, numPoints);
observations = mapPoints.Observations(mapPointIdx, [1, 3]);

for k = 1:numPoints
    % A keyframe shared observed points is not necessarily a connected
    % key frame due to the edge weight constraint of the covisibility graph.   
    % Use intersect to get the correct ViewIds in the tracks
    [viewIds, ia] = intersect(observations{k, 1},refinedKeyFrameIds, 'stable');
    imagePoints   = observations{k, 2}(ia,:);
    tracks(k)     = pointTrack(viewIds, imagePoints);
end

xyzPoints  = mapPoints.Locations(mapPointIdx,:);
camPoses   = poses(vSetKeyFrames, refinedKeyFrameIds);

% Refine local key frames and map points
[refinedPoints, refinedPoses, reprojectionErrors] = bundleAdjustment(...
    xyzPoints, tracks, camPoses, intrinsics, 'FixedViewIDs', fixedViewIds, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 100);

maxError   = 6;
isInlier   = reprojectionErrors < maxError;
outlierIdx = mapPointIdx(~isInlier);

% Update map points and key frames
if ~isempty(outlierIdx)
    mapPoints = deleteObservation(mapPoints, outlierIdx, refinedKeyFrameIds);
    mapPoints = updateValidity(mapPoints, outlierIdx, false);
end

mapPoints = updateLocation(mapPoints, refinedPoints(isInlier,:), mapPointIdx(isInlier));
vSetKeyFrames = updateView(vSetKeyFrames, refinedPoses);
mapPoints = updateViewAndRange(mapPoints, vSetKeyFrames.Views, mapPointIdx(isInlier));
end