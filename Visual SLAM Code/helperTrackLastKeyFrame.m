%helperTrackLastKeyFrame Estimate the camera pose by tracking the last key frame
%   [currPose, mapPointIdx, featureIdx] = helperTrackLastKeyFrame(mapPoints, 
%   views, currFeatures, currPoints, lastKeyFrameId, intrinsics) estimates
%   the camera pose of the current frame by matching features with the
%   previous key frame.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.
%
%   Inputs
%   ------
%   mapPoints         - A helperMapPoints objects storing map points
%   views             - View attributes of key frames
%   currFeatures      - Features in the current frame 
%   currPoints        - Feature points in the current frame                 
%   lastKeyFrameId    - ViewId of the last key frame 
%   intrinsics        - Camera intrinsics 
%   
%   Outputs
%   -------
%   currPose          - Estimated camera pose of the current frame
%   mapPointIdx       - Indices of map points observed in the current frame
%   featureIdx        - Indices of features corresponding to mapPointIdx

%   Copyright 2019 The MathWorks, Inc.

function [currPose, mapPointIdx, featureIdx] = helperTrackLastKeyFrame(...
    mapPoints, views, currFeatures, currPoints, lastKeyFrameId, intrinsics)

imageSize    = [480 640];
scaleFactor  = 1.2;

% Match features from the previous key frame with known world locations
[index3d, index2d]    = getProjectionIndexPair(mapPoints, lastKeyFrameId);
lastKeyFrameFeatures  = views.Features{lastKeyFrameId}(index2d,:);
lastKeyFramePoints    = views.Points{lastKeyFrameId}(index2d);

indexPairs  = matchFeatures(currFeatures, binaryFeatures(lastKeyFrameFeatures),...
    'Unique', true, 'MaxRatio', 0.9, 'MatchThreshold', 40);

% Estimate the camera pose
matchedImagePoints = currPoints.Location(indexPairs(:,1),:);
matchedWorldPoints = mapPoints.Locations(index3d(indexPairs(:,2)), :);

matchedImagePoints = cast(matchedImagePoints, 'like', matchedWorldPoints);
[worldOri, worldLoc, inlier] = estimateWorldCameraPose(...
    matchedImagePoints, matchedWorldPoints, intrinsics, ...
    'Confidence', 90, 'MaxReprojectionError', 4, 'MaxNumTrials', 1e4);

currPose = rigid3d(worldOri, worldLoc);

% Refine camera pose only
currPose = bundleAdjustmentMotion(matchedWorldPoints(inlier,:), ...
    matchedImagePoints(inlier,:), currPose, intrinsics, ...
    'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 20);

% Search for more matches with the map points in the previous key frame
xyzPoints = mapPoints.Locations(index3d,:);

[isInImage, projectedPoints] = helperFindProjectedPointsInImage(xyzPoints, ...
    currPose, intrinsics, imageSize);

minScales    = max(1, lastKeyFramePoints.Scale(isInImage)/scaleFactor);
maxScales    = lastKeyFramePoints.Scale(isInImage)*scaleFactor;
r            = 4;
searchRadius = r*lastKeyFramePoints.Scale(isInImage);

indexPairs   = helperMatchFeaturesInRadius(lastKeyFrameFeatures(isInImage,:), ...
    currFeatures.Features, lastKeyFramePoints(isInImage), currPoints, ...
    projectedPoints, searchRadius, minScales, maxScales);

% Obtain the index of matched map points and features
tempIdx            = find(isInImage); % Convert to linear index
indexPairs(:,1)    = tempIdx(indexPairs(:,1));
covisiblePointsIdx = index3d(indexPairs(:,1));
inliers            = mapPoints.Validity(covisiblePointsIdx);
indexPairs         = indexPairs(inliers,:);

mapPointIdx        = covisiblePointsIdx(inliers);
featureIdx         = indexPairs(:,2);

% Refine the camera pose again
matchedWorldPoints = mapPoints.Locations(mapPointIdx, :);
matchedImagePoints = currPoints.Location(featureIdx, :);

currPose = bundleAdjustmentMotion(matchedWorldPoints, matchedImagePoints, ...
    currPose, intrinsics, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
    'RelativeTolerance', 1e-15, 'MaxIteration', 20);
end