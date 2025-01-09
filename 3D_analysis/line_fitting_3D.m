% line parameters (x, v) as:
% x = centroid, and v =vmax/∥vmax∥

function [application_point, line_direction] = line_fitting_3D(point_cloud)

    centroid = mean(point_cloud);
   % Compute the centered coordinates
    centered_points = point_cloud - centroid;
    
   % Compute the covariance matrix
    covariance_matrix = centered_points' * centered_points;
    
    % Compute the eigenvalues and eigenvectors
    [eigenvectors, eigenvalues] = eig(covariance_matrix);
    
    % Extract the eigenvector corresponding to the maximum eigenvalue as the line direction
    [~, max_index] = max(diag(eigenvalues));
    line_direction = eigenvectors(:, max_index);
    line_direction = line_direction / norm(line_direction);
    
    %application point is the centroid
    application_point = centroid;
end
