% RANSAC 3D line fitting
function [line_params, inliers, outliers] = ransac_line_fitting_3D(point_cloud, num_iterations, threshold_distance)
    % Set RANSAC parameters
    max_inliers = 0;
    best_line_params = [];
    best_inliers = [];
    
    % Iterate 
    for iter = 1:num_iterations
        % Randomly select two points as the two points on the line
        idx = randperm(size(point_cloud, 1), 2);
        point1 = point_cloud(idx(1), :);
        point2 = point_cloud(idx(2), :);
        
        % Compute the direction vector of the line
        line_direction = point2 - point1;
        line_direction = line_direction / norm(line_direction);
        
       % Compute the application_point of the line
        application_point = point1;
        
        % Subtract the application point from each point to obtain the vectors
        vec_from_application = point_cloud - repmat(application_point, size(point_cloud, 1), 1);
    
        % Compute the distance from each point to the line
        distances = vecnorm(cross(vec_from_application, repmat(line_direction, size(point_cloud, 1), 1)), 2, 2) ./ vecnorm(line_direction);

        % Compute the current inliers and outliers
        current_inliers = point_cloud(distances < threshold_distance, :);
        current_outliers = point_cloud(distances >= threshold_distance, :);
        
        % Update the best model parameters
        if size(current_inliers, 1) > max_inliers
            max_inliers = size(current_inliers, 1);
            best_line_params = [application_point, line_direction];
            best_inliers = current_inliers;
        end
    end
    
    line_params = best_line_params;
    inliers = best_inliers;
    outliers = setdiff(point_cloud, inliers, 'rows');
end