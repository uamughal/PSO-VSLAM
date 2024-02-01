function [features, featureMetrics, varargout] = helperSURFFeatureExtractorFunction(I)
% helperSURFFeatureExtractorFunction Implements the SURF feature extraction 
% used in bagOfFeatures.
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.

%   Copyright 2019 The MathWorks, Inc.

% Preprocess the Image
grayImage = rgb2gray(I);

% Feature Extraction
points   = detectSURFFeatures(grayImage);
features = extractFeatures(grayImage, points);

% Compute the Feature Metric. Use the variance of features as the metric
featureMetrics = var(features,[],2);

% Optionally return the feature location information. The feature location
% information is used for image search applications. See the retrieveImages
% and indexImages functions.
if nargout > 2
    % Return feature location information
    varargout{1} = points.Location;
end


