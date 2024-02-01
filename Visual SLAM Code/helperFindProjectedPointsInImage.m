function [isInImage, imagePoins] = helperFindProjectedPointsInImage(...
    xyzPoints, cameraPose, intrinsics, imageSize)
%helperFindProjectedPointsInImage check if projected world points are within 
%   an image
%
%   This is an example helper function that is subject to change or removal 
%   in future releases.


%   Copyright 2019 The MathWorks, Inc.

[R, t] = cameraPoseToExtrinsics(cameraPose.Rotation, cameraPose.Translation);

imagePoints = worldToImage(intrinsics, R, t, xyzPoints);

isInImage  = imagePoints(:,1) < imageSize(2) & imagePoints(:,1) > 0 & ...
    imagePoints(:,2) > 0  & imagePoints(:,2) < imageSize(1);

imagePoins = imagePoints(isInImage, :);

end