% plane parameters (n, d) as:
% n =vmin/∥vmin∥  ,  d = n'*centroid.

function [normal_vector, distance_to_origin] = plane_fitting_3D(cloud_3D)

    centroid = mean(cloud_3D);
   % Compute the centered coordinates
    centered_points = cloud_3D - centroid;
    
    % Compute the covariance matrix
    covariance_matrix = centered_points' * centered_points;
    
    % Compute the eigenvalues and eigenvectors
    [eigenvectors, eigenvalues] = eig(covariance_matrix);
    
    % Extract the eigenvector corresponding to the min eigenvalue as the normal
    [min_eigenvalue, min_index] = min(diag(eigenvalues));
    min_eigenvector = eigenvectors(:, min_index);
    normal_vector = min_eigenvector / norm(min_eigenvector);
    
    % calculate distance_to_origin
    distance_to_origin = dot(normal_vector', centroid);
end