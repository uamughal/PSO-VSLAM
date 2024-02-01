function isStrong = helperSelectStrongConnections(connections, viewIds, currViewId, threshold)
%helperSelectStrongConnections select strong connections with more than
%   a specified number of matches
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019 The MathWorks, Inc.

[~,ia] = intersect(connections{:,1:2}, ...
    [viewIds, currViewId*ones(numel(viewIds),1,'uint32')],'stable','row');

isStrong = cellfun(@(x) size(x, 1) > threshold, connections.Matches(ia));
