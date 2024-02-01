classdef helperVisualizeMotionAndStructure < handle
%helperVisualizeMatchedFeatures show map points and camera trajectory
%
%   This is an example helper class that is subject to change or removal 
%   in future releases.

%   Copyright 2019 The MathWorks, Inc.

    properties
        XLim = [-1.5 1.5]
        
        YLim = [-1 0.5]
        
        ZLim = [-0.5 2]
        
        Axes
    end
    
    properties (Access = private)
        MapPointsPlot
 
        EstimatedTrajectory
        
        OptimizedTrajectory

        CameraPlot
    end
    
    methods (Access = public)
        function obj = helperVisualizeMotionAndStructure(vSetKeyFrames, mapPoints)
        
            [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
             
            obj.MapPointsPlot = pcplayer(obj.XLim, obj.YLim, obj.ZLim, ...
                'VerticalAxis', 'y', 'VerticalAxisDir', 'down', 'MarkerSize', 30);
            
            obj.Axes  = obj.MapPointsPlot.Axes;
            obj.MapPointsPlot.view(xyzPoints); 
            obj.Axes.Children.DisplayName = 'Map points';
            
            hold(obj.Axes, 'on');
            
            % Set figure position on the screen
            movegui(obj.Axes.Parent, [1000 200]);
            
            % Plot camera trajectory
            obj.EstimatedTrajectory = plot3(obj.Axes, trajectory(:,1), trajectory(:,2), ...
                trajectory(:,3), 'r', 'LineWidth', 2 , 'DisplayName', 'Estimated trajectory');
            
            % Plot the current cameras
            obj.CameraPlot = plotCamera(currPose, 'Parent', obj.Axes, 'Size', 0.05);
        end
        
        function updatePlot(obj, vSetKeyFrames, mapPoints)
            
            [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints);
            
            % Update the point cloud
            obj.MapPointsPlot.view(xyzPoints);
            
            % Update the camera trajectory
            set(obj.EstimatedTrajectory, 'XData', trajectory(:,1), 'YData', ...
                trajectory(:,2), 'ZData', trajectory(:,3));
            
            % Update the current camera pose since the first camera is fixed
            obj.CameraPlot.AbsolutePose = currPose.AbsolutePose;
            obj.CameraPlot.Label        = num2str(currPose.ViewId);
            
            drawnow limitrate
        end
        
        function plotOptimizedTrajectory(obj, poses)
            
            % Delete the camera plot
            delete(obj.CameraPlot);
            
            % Plot the optimized trajectory
            trans = vertcat(poses.AbsolutePose.Translation);
            obj.OptimizedTrajectory = plot3(obj.Axes, trans(:, 1), trans(:, 2), trans(:, 3), 'm', ...
                'LineWidth', 2, 'DisplayName', 'Optimized trajectory');
        end
        
        function plotActualTrajectory(obj, gTruth, optimizedPoses)
            estimatedCams = vertcat(optimizedPoses.AbsolutePose.Translation);
            actualCams    = vertcat(gTruth.Translation);
            scale = median(vecnorm(actualCams, 2, 2))/ median(vecnorm(estimatedCams, 2, 2));
            
            % Update the plot based on the ground truth
            updatePlotScale(obj, scale);
            
            % Plot the ground truth
            plot3(obj.Axes, actualCams(:,1), actualCams(:,2), actualCams(:,3), ...
                'g','LineWidth',2, 'DisplayName', 'Actual trajectory');
            
            drawnow limitrate
        end
        
        function showLegend(obj)
            % Add a legend to the axes
            hLegend = legend(obj.Axes, 'Location',  'northeast', ...
                'TextColor', [1 1 1], 'FontWeight', 'bold');
        end
    end
    
    methods (Access = private)
        function [xyzPoints, currPose, trajectory]  = retrievePlottedData(obj, vSetKeyFrames, mapPoints)
            camPoses    = poses(vSetKeyFrames);
            currPose    = camPoses(end,:); % Contains both ViewId and Pose
            trajectory  = vertcat(camPoses.AbsolutePose.Translation);
            xyzPoints   = mapPoints.Locations(mapPoints.Validity,:);
            
            % Only plot the points within the limit
            inPlotRange = xyzPoints(:, 1) > obj.XLim(1) & ...
                xyzPoints(:, 1) < obj.XLim(2) & xyzPoints(:, 2) > obj.YLim(1) & ...
                xyzPoints(:, 2) < obj.YLim(2) & xyzPoints(:, 3) > obj.ZLim(1) & ...
                xyzPoints(:, 3) < obj.ZLim(2);
            xyzPoints   = xyzPoints(inPlotRange, :);
        end
        
        function updatePlotScale(obj, scale)
            % Update the map points and camera trajectory based on the
            % ground truth scale
            obj.Axes.XLim = obj.Axes.XLim * scale;
            obj.Axes.YLim = obj.Axes.YLim * scale;
            obj.Axes.ZLim = obj.Axes.ZLim * scale;
            
            % Map points
            obj.Axes.Children(end).XData = obj.Axes.Children(end).XData * scale;
            obj.Axes.Children(end).YData = obj.Axes.Children(end).YData * scale;
            obj.Axes.Children(end).ZData = obj.Axes.Children(end).ZData * scale;
            
            % Estiamted and optimized Camera trajectory
            obj.EstimatedTrajectory.XData =  obj.EstimatedTrajectory.XData * scale;
            obj.EstimatedTrajectory.YData =  obj.EstimatedTrajectory.YData * scale;
            obj.EstimatedTrajectory.ZData =  obj.EstimatedTrajectory.ZData * scale;
            obj.OptimizedTrajectory.XData =  obj.OptimizedTrajectory.XData * scale;
            obj.OptimizedTrajectory.YData =  obj.OptimizedTrajectory.YData * scale;
            obj.OptimizedTrajectory.ZData =  obj.OptimizedTrajectory.ZData * scale;
        end
    end
end

