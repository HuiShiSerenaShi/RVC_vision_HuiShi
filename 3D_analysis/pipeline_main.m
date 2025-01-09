clear all
close all
clc

% Read the RGB and depth images
image = imread('milkbox.jpg'); 
depth_image = imread('milkbox.png'); 
figure;
imshow(image);
title('RGB Image');
figure;
imagesc(depth_image);
colormap parula; 
colorbar; 
title('Depth Image');

% Set depth thresholds
depth_threshold_upper = 643; 
depth_threshold_lower = 600;
% Generate masks based on thresholds
box_mask = (depth_image > depth_threshold_lower) & (depth_image < depth_threshold_upper);
background_mask = ~box_mask;

depth_max = max(depth_image(:));
depth_min = min(depth_image(:));

thresholded = depth_image;
% Get thresholded image, mark box region in depth image as yellow, background as deep blue
thresholded(box_mask) = depth_max; 
thresholded(background_mask) = depth_min;
figure;
imagesc(thresholded);
title('Thresholded Depth Image');

% Binarized Image
BW = imbinarize(thresholded);
BW2 = bwareaopen(BW,50);
figure;
imshow(BW2);
title('Binarized Image');

% Calculate connected components 
cc = bwconncomp(BW2);
numObjects = cc.NumObjects;
disp(['Number of connected components: ', num2str(numObjects)]);

% Get region properties
stats = regionprops(cc, 'Centroid', 'BoundingBox', 'Orientation', 'Extrema');

centroid = cat(1, stats.Centroid); 
boundingBox = cat(1, stats.BoundingBox); 
orientation_a = [stats.Orientation]; 
extrema = [stats.Extrema]; 
orientation_rad = deg2rad(orientation_a);
orientation_vec = [cos(orientation_rad) -sin(orientation_rad)];

disp('Centroid:');
disp(centroid);
disp('Bounding Box:');
disp(boundingBox);
disp('Orientation a:');
disp(orientation_a);
disp('Extrema:');
disp(extrema);

% find obj orientation
% Calculate arrow endpoints 
arrow_endpoints_x = centroid(:,1) + orientation_vec(:,1) * 20; % Scale vector length to 20
arrow_endpoints_y = centroid(:,2) + orientation_vec(:,2) * 20; 
P_on_orientation = [centroid(:,1) + orientation_vec(:,1) * 36
centroid(:,2) + orientation_vec(:,2) * 36
]
% Plot orientation
figure;
imshow(BW2);
hold on;
plot(centroid(1), centroid(2), 'r.', 'MarkerSize', 10, 'LineWidth', 2);
quiver(centroid(:,1), centroid(:,2), arrow_endpoints_x - centroid(:,1), arrow_endpoints_y - centroid(:,2), 0, 'Color', 'g', 'LineWidth', 1);
plot(P_on_orientation(1), P_on_orientation(2), 'b.', 'MarkerSize', 10, 'LineWidth', 2);

% Plot contours
[C, ~] = contour(BW2, [0.5, 0.5], 'r', 'LineWidth', 2); 

title('2D Contour Visualization');
hold off;
C = C(:,2:end);

% Depth image Boundary Only
depth_boundary_only = zeros(size(thresholded));
for i = 1:size(C,2)
    x = round(C(1,i));
    y = round(C(2,i));
    depth_boundary_only(y,x) = depth_max;
end
% figure;
% imagesc(depth_boundary_only);
% colormap parula; 
% colorbar; 
% title('Depth Boundary Only');

% Set camera parameters 
fu = 525;          
fv = 525;           
uo = 319.5;         
vo = 239.5;        
% Calculate obj cloud_3D
[cloud_3D, X, Y] = range_to_cloud(thresholded, fu, fv, uo, vo);
[boundary_3D,~,~] = range_to_cloud(depth_boundary_only, fu, fv, uo, vo);
centroid_3D = mean(cloud_3D);
ind_x = round(P_on_orientation(2));
ind_y = round(P_on_orientation(1));
P3D_on_orientation = [X(ind_x,ind_y) Y(ind_x,ind_y) thresholded(ind_x,ind_y)];

% 3D plane_fitting
% vec_orientation = centroid_3D- double(P3D_on_orientation) ;
[normal_vector, distance_to_origin] = plane_fitting_3D(cloud_3D);
% third_axis = cross(normal_vector, vec_orientation);
normal_vector = normal_vector / norm(normal_vector); 
% vec_orientation = vec_orientation / norm(vec_orientation);
% third_axis = third_axis / norm(third_axis);

figure;
hold on;
plot3(cloud_3D(:,1), cloud_3D(:,2), cloud_3D(:,3), 'b.', 'MarkerSize', 5); 
plot3(boundary_3D(:,1), boundary_3D(:,2), boundary_3D(:,3), 'r.', 'MarkerSize', 5); 
plot3(centroid_3D(:,1), centroid_3D(:,2), centroid_3D(:,3), 'r.', 'MarkerSize', 30); 
plot3(P3D_on_orientation(:,1), P3D_on_orientation(:,2), P3D_on_orientation(:,3), 'g.', 'MarkerSize', 25); 
% quiver3(centroid_3D(1), centroid_3D(2), centroid_3D(3), normal_vector(1), normal_vector(2), normal_vector(3), 'r', 'LineWidth', 2);
% quiver3(centroid_3D(1), centroid_3D(2), centroid_3D(3), vec_orientation(1), vec_orientation(2), vec_orientation(3), 'r', 'LineWidth', 5);
% quiver3(centroid_3D(1), centroid_3D(2), centroid_3D(3), third_axis(1), third_axis(2), third_axis(3), 'r', 'LineWidth', 5);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point Cloud Visualization');
view(3);
grid on;
hold off;

% Plot the fitted plane on the point cloud
x_range = [min(cloud_3D(:,1)), max(cloud_3D(:,1))];
y_range = [min(cloud_3D(:,2)), max(cloud_3D(:,2))];
[X_plane, Y_plane] = meshgrid(x_range, y_range);
%Z=(−Ax−By-D)/C  , A = nx, B = ny, C = nz, D = −d
Z_plane = (-normal_vector(1) * X_plane - normal_vector(2) * Y_plane + distance_to_origin) / normal_vector(3);

% Plot projected_boundary on plane
projected_boundary = [];
for i = 1:size(boundary_3D, 1)
  projected = point3D_to_plane_projection(boundary_3D(i,:), normal_vector, distance_to_origin);
  projected_boundary = [projected_boundary; projected];
end

figure;
hold on;
plot3(cloud_3D(:,1), cloud_3D(:,2), cloud_3D(:,3), 'b.');
plot3(boundary_3D(:,1), boundary_3D(:,2), boundary_3D(:,3), 'r.', 'MarkerSize', 5); 
plot3(centroid_3D(:,1), centroid_3D(:,2), centroid_3D(:,3), 'r.', 'MarkerSize', 30); 
plot3(projected_boundary(:,1), projected_boundary(:,2), projected_boundary(:,3), 'g.', 'MarkerSize', 5);
surf(X_plane, Y_plane, Z_plane, 'FaceAlpha', 0.5, 'FaceColor', 'r', 'EdgeColor', 'none');
% Plot the normal vector with adjusted length
quiver3(centroid_3D(1), centroid_3D(2), centroid_3D(3), normal_vector(1) * 5, normal_vector(2) * 5, normal_vector(3) * 5, 'r', 'LineWidth', 2); 
view(3);
hold off;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Fitted Plane and Point Cloud');


figure;
hold on;
% Line fitting using RANSAC and find intersections, and plot them
plot3(cloud_3D(:,1), cloud_3D(:,2), cloud_3D(:,3), 'b.');
plot3(boundary_3D(:,1), boundary_3D(:,2), boundary_3D(:,3), 'g.', 'MarkerSize', 5);
num_iterations=500;
threshold_distance = 10;
outliers=boundary_3D;
line_params=[];
colormap_lines = lines(4);
for i=1:4
    % Fit lines using RANSAC and use outliers from previous loop
    [line_params_, inliers, outliers_] = ransac_line_fitting_3D(outliers, num_iterations, threshold_distance)
    line_params = [line_params;line_params_];
    outliers=outliers_;
    line_direction = line_params_(4:6);
    application_point = line_params_(1:3);
    % Plot line
    max_t = 2000; 
    t = linspace(-max_t, max_t, 800);
    line_points = repmat(application_point, [length(t), 1]) + t' * line_direction;
    plot3(line_points(:,1), line_points(:,2), line_points(:,3), 'Color', colormap_lines(i,:), 'LineWidth', 2);

end

%  Find line intersections 
[intersection_points, valid_pairs] = find_line_intersections(line_params);
%  Plot intersections
plot3(centroid_3D(:,1), centroid_3D(:,2), centroid_3D(:,3), 'r.', 'MarkerSize', 30); 
plot3(intersection_points(:,1), intersection_points(:,2), intersection_points(:,3), 'g.', 'MarkerSize', 30); 
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Point Cloud with Fitted Line');
view(3);
grid on;
hold off;