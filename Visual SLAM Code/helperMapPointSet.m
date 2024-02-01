% helperMapPointSet Object for managing map data in structure-from-motion and visual SLAM
%   Use this object to store map points attributes, such as world location,
%   view direction and predicted depth range, and 3D-to-2D correspondences
%   between the map points and the views observing the map points.
%
%   This is an example helper class that is subject to change or removal 
%   in future releases.
%
%   mapPoints = helperMapPointSet() returns a helperMapPointSet object. 
%   
%   mapPoints = helperMapPointSet(Name, Value) specifies additional  
%   name-value pair arguments as described below:
%
%   'MaxNumPoints'      - Maximum number of map points stored in the object
%                           
%                         Default:  2e4
%
%   'MaxNumViews'       - Maximum number of views associated with the map
%                         points
%
%                         Default:  500
%
%   helperMapPointSet properties:
%   Locations           - An M-by-3 matrix representing the [X, Y, Z] locations
%                         of map points in the world coordinates
%   ViewDirections      - An M-by-3 matrix representing the view direction
%                         of each map point
%   MaxDistance         - An M-by-1 vector representing the maximum scale 
%                         invariant distance for each map point
%   MinDistance         - An M-by-1 vector representing the minimum scale 
%                         invariant distance for each map point
%   Validity            - An M-by-1 vector representing the logical indices 
%                         for valid map points
%   Observations        - A 5-column cell array storing 3D-to-2D correspondences.
%                         In the K-th row, where K is the linear index of 
%                         a map point, the first column contains a vector of 
%                         ViewIds denoting the views observing the map point. 
%                         The second column contains the indices of features
%                         corresponding to the map point in each view. 
%                         The third column contains the corresponding [x, y] 
%                         locations of feature points in each view. The fourth
%                         column contains the corresponding scales of the
%                         features points. The fifth column contains the  
%                         index of the ViewIds in the first column that   
%                         denotes the view containing the representative 
%                         features of the map point
%   NumPoints           - An integer representing the number of map points
%                         (read-only)
%
%   helperMapPointSet methods:
%
%   addMapPoint             - Add map points with [X, Y, Z] locations
%   addObservation          - Attach ViewIds and feature indices to map points
%   deleteObservation       - Remove ViewIds and feature indices for map points
%   getMapPointIndex        - Get the indices of map point observed in a view 
%   getFeatureIndex         - Get the indices of feature points in a view
%                             with known corresponding map points
%   getProjectionIndexPair  - Get the indices of map points observed in a view  
%                             and the indices of corresponding feature points
%   updateLocation          - Update the [X, Y, Z] locations of map points
%   updateViewAndRange      - Update the view direction and the predicted 
%                             depth range of map points
%   updateValidity          - Update the validity of map points

%   Copyright 2019 The MathWorks, Inc.

classdef helperMapPointSet
    
    properties (GetAccess = public, SetAccess = public)
        %Locations 
        %
        Locations
        
        %ViewDirections 
        ViewDirections
        
        %Validity 
        Validity
        
        %MinDistance
        MinDistance
        
        %MaxDistance
        MaxDistance
        
        %Observations
        Observations
    end
    
    properties (GetAccess = public, SetAccess = private)
        %NumPoints 
        NumPoints    = 0
    end
    
    properties
        %Projections A 2-column cell array storing 3D-to-2D projections
        %
        %   In the K-th row, where K is the ViewId, the first column contains
        %   the indices of the map points observed in the view specified 
        %   by view K. The second column contains the indices of corresponding
        %   features in view K
        %
        Projections
    end
    
    methods
        %------------------------------------------------------------------
        function obj = helperMapPointSet(varargin)

            % Instantiate an input parser
            parser = inputParser;
            
            % Specify the optional parameters
            parser.addParameter('MaxNumPoints', 2e4);
            parser.addParameter('MaxNumViews',  500);
            
            % Parse
            parser.parse(varargin{:});
            r = parser.Results;
            
            % Preallocating for better performance
            obj.Observations  = cell(r.MaxNumPoints, 5);
            obj.Projections   = cell(r.MaxNumViews,  2);
        end
        
        %------------------------------------------------------------------
        function [obj, indices] = addMapPoint(obj, Locations)
            
            newNumPoints  = size(Locations, 1);
            
            % Return indices as a column vector
            indices = (obj.NumPoints+1:obj.NumPoints+newNumPoints).'; 
            
            obj.NumPoints = obj.NumPoints + newNumPoints;
            
            % Set [X, Y, Z] locations
            obj.Locations(indices, :) = Locations;
            
            if isempty(obj.Validity)
                obj.Validity = true(newNumPoints, 1); % Logic index
            else
                obj.Validity(indices, 1)  = true;
            end
        end
        
        function obj = updateLocation(obj, xyzPoints, varargin)
            
            if nargin > 2
                % Update part of map points
                indices = varargin{1};
                obj.Locations(indices,:) = xyzPoints;
            else
                % Update all the map points
                obj.Locations = xyzPoints;
            end
        end
        
        %------------------------------------------------------------------
        function obj = updateViewAndRange(obj, views, mapPointsIndices)
            
            % Extract the columns for faster query
            viewsLocations= vertcat(views.AbsolutePose.Translation);
            viewsFeatures = views.Features;
            viewsPoints   = views.Points;
            observations  = obj.Observations;
            
            % Update for each map point
            for j = 1: numel(mapPointsIndices)
                pointIdx      = mapPointsIndices(j);
                
                observation   = obj.Observations(pointIdx, :);
                keyFrameIds   = observation{1};
                numCameras    = numel(keyFrameIds);
                
                % Update mean viewing direction
                allFeatures   = zeros(numCameras, 32, 'uint8');
                allFeatureIdx = observation{2};

                for k = 1:numCameras
                    tempId            = keyFrameIds(k);
                    features          = viewsFeatures{tempId};
                    featureIndex      = allFeatureIdx(k);
                    allFeatures(k, :) = features(featureIndex, :);
                end
                
                directionVec = obj.Locations(pointIdx,:) - viewsLocations(keyFrameIds,:);
                
                % Update view direction
                meanViewVec = mean(directionVec ./ (vecnorm(directionVec, 2, 2)), 1); 
                obj.ViewDirections(pointIdx, :) = meanViewVec/norm(meanViewVec);
                
                % Identify the distinctive descriptor and the associated key frame
                distIndex = computeDistinctiveDescriptors(allFeatures);
                
                % Update the distinctive key frame index
                obj.Observations{pointIdx, 5} = distIndex;
                
                % Update depth range
                distDirectionVec = directionVec(distIndex,:);
                distKeyFrameId   = keyFrameIds(distIndex);
                allScales        = obj.Observations{pointIdx,4};
                maxDist          = norm(distDirectionVec)* allScales(distIndex);

                points           = viewsPoints{distKeyFrameId}; 
                scales           = points.Scale;
                minDist          = maxDist/max(scales);
                
                obj.MaxDistance(pointIdx, 1) = maxDist;
                obj.MinDistance(pointIdx, 1) = minDist;
            end
        end
        
        
        %------------------------------------------------------------------
        function obj = addObservation(obj, mapPointsIndices, viewId, ...
                featureIndices, locations, scales)
            
            viewId           = cast(viewId, 'uint32');
            mapPointsIndices = cast(mapPointsIndices, 'uint32');
            featureIndices   = cast(featureIndices,'uint32');
            
            for i = 1:numel(mapPointsIndices)
                idx = mapPointsIndices(i);
                existingIds = obj.Observations{mapPointsIndices(i), 1};
                numKeyIDs = numel(existingIds);
                
                % add new observation: map points -> views
                obj.Observations{idx, 1}(numKeyIDs+1,1) = viewId;
                obj.Observations{idx, 2}(numKeyIDs+1,:) = featureIndices(i);
                obj.Observations{idx, 3}(numKeyIDs+1,:) = locations(i,:); % [x, y]
                obj.Observations{idx, 4}(numKeyIDs+1,:) = scales(i); 
            end
            
            % Update projections: views -> map points
            if isempty(obj.Projections{viewId, 1})
                obj.Projections{viewId, 1} = mapPointsIndices;
                obj.Projections{viewId, 2} = featureIndices;
            else
                obj.Projections{viewId, 1} = [obj.Projections{viewId, 1}; mapPointsIndices];
                obj.Projections{viewId, 2} = [obj.Projections{viewId, 2}; featureIndices];
            end
        end
        
        %------------------------------------------------------------------
        function obj = deleteObservation(obj, mapPointsIndices, viewIds)

            for i = 1:numel(mapPointsIndices)
                idx = mapPointsIndices(i);
                oldViewIds = obj.Observations{idx, 1};
                [~, ia] = setdiff(oldViewIds, viewIds, 'stable');
                if ~isempty(ia)
                    obj.Observations{idx, 1} = obj.Observations{idx, 1}(ia);
                    obj.Observations{idx, 2} = obj.Observations{idx, 2}(ia);
                    obj.Observations{idx, 3} = obj.Observations{idx, 3}(ia, :);
                    obj.Observations{idx, 4} = obj.Observations{idx, 4}(ia, :);
                end
            end
            
            for j = 1:numel(viewIds)
                oldMapPointsIndices = obj.Projections{viewIds(j), 1};
                [~, ia] = setdiff(oldMapPointsIndices, mapPointsIndices, 'stable');
                if ~isempty(ia)
                    obj.Projections{viewIds(j), 1} = obj.Projections{viewIds(j), 1}(ia);
                    obj.Projections{viewIds(j), 2} = obj.Projections{viewIds(j), 2}(ia);
                end
            end
        end
        
        %------------------------------------------------------------------
        function obj = updateValidity(obj, indices, values)
            
            validateattributes(values, {'logical'}, {'vector'});
            
            obj.Validity(indices) = values;
        end
        
        %------------------------------------------------------------------
        function index3d = getMapPointIndex(obj, viewIds)
            if isscalar(viewIds)
                index3d = obj.Projections{viewIds, 1};
            else
                index3d = unique(vertcat(obj.Projections{viewIds, 1}));
            end
        end
        
        %------------------------------------------------------------------
        function index2d = getFeatureIndex(obj, viewId)
            index2d = obj.Projections{viewId, 2};
        end
        
        %------------------------------------------------------------------
        function [index3d, index2d] = getProjectionIndexPair(obj, viewId)
            index3d = getMapPointIndex(obj, viewId);
            index2d = getFeatureIndex(obj, viewId);
        end
    end

end

%------------------------------------------------------------------
function index = computeDistinctiveDescriptors(features)
%computeDistinctiveDescriptors Find the distinctive discriptor

if size(features, 1) < 3
    index       = 2;
else
    scores      = helperHammingDistance(features, features);
    [~, index]  = min(sum(scores, 2));
end
end
