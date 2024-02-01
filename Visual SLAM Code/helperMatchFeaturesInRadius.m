function indexPairs = helperMatchFeaturesInRadius(features1, features2, ...
    points1, points2, projectedPoints, radius, minScales, maxScales)
%helperMatchFeaturesInRadius Match features within a radius 
%   indexPairs = helperMatchFeaturesInRadius(feature1, feature2, points1,  
%   points2, projectedPoints, radius, minScales, maxScales) returns a 
%   P-by-2 matrix, indexPairs, containing the indices to the features most
%   likely to correspond between the two input feature matrices satisfying
%   the distance and the scale constraints. 
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.
%
%   Inputs
%   ------
%   features1              - Feature matrices in the first image
%   features2              - Feature matrices in the second image
%   points1                - Feature points corresponding to features1
%   points2                - Feature points corresponding to features2
%   projectedPoints        - The projection in the second image of the 
%                            world points corresponding to points1
%   radius                 - Searching radius
%   minScales              - Minimum scales of feature points points1  
%   maxScales              - Maximum scales of feature points points1
%
%   Output
%   ------
%   indexPairs             - Indices of corresponding features 

%   Copyright 2019 The MathWorks, Inc.

matchThreshold = 100;
maxRatio       = 0.8;
scaleFactor    = 1.2;

numPoints      = size(projectedPoints, 1);
indexPairs     = zeros(numPoints, 2, 'uint32');
scales1        = points1.Scale;
scales2        = points2.Scale;
neighborPoints = points2.Location;

% Convert scales to number of levels
neighborLevels = round(log(scales2)/log(scaleFactor));
minLevels      = round(log(minScales)/log(scaleFactor));
maxLevels      = round(log(maxScales)/log(scaleFactor));

if isscalar(radius)
    radius = radius * ones(numPoints, 1);
end

numMatches = 1;

for i = 1: numPoints
    % Find points within a radius subjected to the scale constraint
    pointIdx = findPointsInRadius(neighborPoints, projectedPoints(i,:), ...
        radius(i), neighborLevels, minLevels(i), maxLevels(i));
    
    if ~isempty(pointIdx)
        centerFeature   = features1(i,:);
        nearbyFeatures  = features2(pointIdx,:);
        nearbyLevels    = neighborLevels(pointIdx);
        
        if numel(pointIdx) == 1
            bestIndex = pointIdx;
        else
            scores = helperHammingDistance(centerFeature, nearbyFeatures);
            
            % Find the best two matches
            [minScore, index] = mink(scores, 2);
            if minScore(1) < matchThreshold
                % Ratio test when the best two matches have the same scale
                if  nearbyLevels(index(1)) == nearbyLevels(index(2)) && ...
                        minScore(1) > maxRatio * minScore(2)
                    continue
                else
                    bestIndex  = pointIdx(index(1));
                end
            else
                continue
            end
        end
        
        % Unique matching pairs
        if ~ismember(bestIndex, indexPairs(:,2))
            indexPairs(numMatches,:) = uint32([i, bestIndex]);
            numMatches = numMatches+1;
        end
    end
end

indexPairs = indexPairs(1:numMatches-1,:);
end

function index = findPointsInRadius(neighborPoints, centerPoint, radius, ...
    neighborLevels, minLevels, maxLevels)

sqrDist   = sum((neighborPoints - centerPoint).^2 , 2);
index     = find(sqrDist < radius^2 & neighborLevels >= minLevels & neighborLevels <= maxLevels);
end
