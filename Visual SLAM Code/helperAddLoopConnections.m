function [isLoopClosed, mapPoints, vSetKeyFrames] = helperAddLoopConnections(...
    mapPoints, vSetKeyFrames, loopCandidates, currKeyFrameId, currFeatures, currPoints, intrinsics)
%helperAddLoopConnections add connections between the current key frame and 
%   the valid loop candidate key frames. A loop candidate is valid if it has 
%   enough covisible map points with the current key frame.

%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019 The MathWorks, Inc.

scaleFactor     = 1.2;
imageSize       = [480, 640];

% Minimum number of matched features for loop edge
minNumMatches   = 60;

numCandidates   = size(loopCandidates,1);
loopConnections = [];
[index3d1, index2d1] = getProjectionIndexPair(mapPoints, currKeyFrameId);
validFeatures1  = currFeatures.Features(index2d1, :);
validPoints1    = currPoints(index2d1).Location;

for k = numCandidates : -1 : 1
    [index3d2, index2d2] = getProjectionIndexPair(mapPoints, loopCandidates(k));
    allFeatures2 = vSetKeyFrames.Views.Features{loopCandidates(k)};
    validFeatures2 = allFeatures2(index2d2, :);
    allPoints2 = vSetKeyFrames.Views.Points{loopCandidates(k)};
    validPoints2 = allPoints2(index2d2);
    
    indexPairs = matchFeatures(binaryFeatures(validFeatures1), binaryFeatures(validFeatures2), ...
        'Unique', true, 'MaxRatio', 0.9, 'MatchThreshold', 90);
    
    % Check if all the candidate key frames have strong connection with the
    % current keyframe
    if size(indexPairs, 1) < minNumMatches
        isLoopClosed = false;
        return
    end
    
    % Estimate the relative pose of the current key frame with respect to the
    % loop candidate keyframe with the highest similarity score
    if k == 1 
        worldPoints = mapPoints.Locations(index3d2(indexPairs(:,2)),:);
        
        matchedImagePoints = cast(validPoints1(indexPairs(:,1),:), 'like', worldPoints);
        [worldOrientation, worldLocation] = estimateWorldCameraPose(matchedImagePoints, worldPoints, intrinsics, ...
            'Confidence', 90, 'MaxReprojectionError', 6, 'MaxNumTrials', 1e4);
        cameraPose = rigid3d(worldOrientation, worldLocation);

        [R, t]    = cameraPoseToExtrinsics(cameraPose.Rotation, cameraPose.Translation);
        xyzPoints = mapPoints.Locations(index3d2,:);
        projectedPoints = worldToImage(intrinsics, R, t, xyzPoints);
        
        isInImage = find(projectedPoints(:,1)<imageSize(2) & projectedPoints(:,1)>0 & ...
            projectedPoints(:,2)< imageSize(1) & projectedPoints(:,2)>0);
        
        minScales    = validPoints2.Scale(isInImage)/scaleFactor;
        maxScales    = validPoints2.Scale(isInImage)*scaleFactor;
        r            = 3;
        searchRadius = r*validPoints2.Scale(isInImage);
        
        matchedIndexPairs = helperMatchFeaturesInRadius(validFeatures2(isInImage,:), currFeatures.Features, ...
            validPoints2(isInImage), currPoints, projectedPoints(isInImage,:), searchRadius, minScales, maxScales);
        matchedIndexPairs(:,1) = isInImage(matchedIndexPairs(:,1));
        
        visiblePointsIndex = index3d2(matchedIndexPairs(:,1));
        validWorldPoints   = mapPoints.Locations(visiblePointsIndex, :);
        matchedImagePoints = currPoints.Location(matchedIndexPairs(:,2),:);
        
        % Refine the pose
        cameraPose = bundleAdjustmentMotion(validWorldPoints, matchedImagePoints, ...
            cameraPose, intrinsics, 'PointsUndistorted', true, 'AbsoluteTolerance', 1e-7,...
            'RelativeTolerance', 1e-15, 'MaxIteration', 50);
        
        % Fuse covisible map points
        [matchedIndex2d1, ia1, ib1] = intersect(index2d1, matchedIndexPairs(:,2), 'stable');
        matchedIndex3d1 = index3d1(ia1);

        matchedIndex3d2 = index3d2(matchedIndexPairs(ib1,1));
        matchedIndex2d2 = index2d2(matchedIndexPairs(ib1,1));
        
        mapPoints = updateLocation(mapPoints, mapPoints.Locations(matchedIndex3d2, :), matchedIndex3d1);
        
        % Add connection between the current key frame and the loop key frame
        pose1   = vSetKeyFrames.Views.AbsolutePose(loopCandidates(k));
        pose2   = cameraPose;
        relPose = rigid3d(pose2.Rotation*pose1.Rotation', (pose2.Translation-pose1.Translation)*pose1.Rotation');
        matches = [matchedIndex2d2, matchedIndex2d1];
        vSetKeyFrames = addConnection(vSetKeyFrames, loopCandidates(k), currKeyFrameId, relPose, 'Matches', matches);

        disp(['Loop edge added between keyframe: ', num2str(loopCandidates(k)), ' and ', num2str(currKeyFrameId)]);
        
        % Add connections between the current key frame and the connected 
        % key frames of the loop key frame
        neighborViews = connectedViews(vSetKeyFrames, loopCandidates(k));
        for m = 1:numel(neighborViews.ViewId)
            neighborViewId = neighborViews.ViewId(m);
            [index3d3, index2d3] = getProjectionIndexPair(mapPoints, neighborViewId);
            [covPointsIndices, ia2, ib2] = intersect(index3d3, matchedIndex3d2, 'stable'); 
            if numel(covPointsIndices) > minNumMatches
                pose1   = neighborViews.AbsolutePose(m);
                pose2   = cameraPose;
                relPose = rigid3d(pose2.Rotation*pose1.Rotation', (pose2.Translation-pose1.Translation)*pose1.Rotation'); 
                matches = [index2d3(ia2), matchedIndex2d1(ib2)];
                if ~hasConnection(vSetKeyFrames, neighborViewId, currKeyFrameId)
                    vSetKeyFrames = addConnection(vSetKeyFrames, neighborViewId, currKeyFrameId, relPose, 'Matches', matches);
                end
                disp(['Loop edge added between keyframe: ', num2str(neighborViewId), ' and ', num2str(currKeyFrameId)]);
            end
        end
        
        isLoopClosed = true;
    end
end
end