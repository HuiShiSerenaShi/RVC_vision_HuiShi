% test some functions
clc
clear all
close all
% point3D_to_plane_distance
point = [1, 2, 3];
plane_normal = [0.5, 0.5, 0.5];
plane_d = 5;

distance = point3D_to_plane_distance(point, plane_normal, plane_d);

figure;
hold on;
plot3(point(1), point(2), point(3), 'r.', 'MarkerSize', 10);
[X_plane, Y_plane] = meshgrid(-10:10, -10:10);
Z_plane = (-plane_normal(1) * X_plane - plane_normal(2) * Y_plane + plane_d) / plane_normal(3);
surf(X_plane, Y_plane, Z_plane, 'FaceAlpha', 0.5, 'FaceColor', 'b', 'EdgeColor', 'none');

projection_point = point - distance * plane_normal;
plot3([point(1), projection_point(1)], [point(2), projection_point(2)], [point(3), projection_point(3)], 'g--', 'LineWidth', 2);
text(point(1), point(2), point(3), ['  Distance = ', num2str(distance)], 'Color', 'g', 'FontSize', 12);
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Point to Plane Distance');
axis equal;
grid on;
view(3);
hold off;


% line_fitting_3D
num_points = 100;
line_length = 10;
noise_level = 0.5;

% Generate points along the line
line_points = linspace(-line_length/2, line_length/2, num_points)';
line_cloud = [line_points, zeros(num_points, 2)]; % Assume the line is along the x-axis
% Add noise to the line points
noisy_line_cloud = line_cloud + noise_level * randn(num_points, 3);

[application_point, line_direction] = line_fitting_3D(noisy_line_cloud);

figure;
hold on;

scatter3(noisy_line_cloud(:,1), noisy_line_cloud(:,2), noisy_line_cloud(:,3), 'b.');
plot3(application_point(1), application_point(2), application_point(3), 'r.', 'MarkerSize', 10);
quiver3(application_point(1), application_point(2), application_point(3), ...
        line_direction(1), line_direction(2), line_direction(3), ...
        'Color', 'g', 'LineWidth', 2);

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Line Fitting in 3D');
axis equal;
grid on;
view(3);
hold off;


% point3D_to_line_projection

line_point = [0, 0, 0]; % Line passes through the origin
line_direction = [1, 1, 1]; % Direction of the line

num_points = 10;
point_cloud = rand(num_points, 3) * 10; % Random points in a 10x10x10 cube

projection_points = zeros(num_points, 3);
distances = zeros(num_points, 1);
for i = 1:num_points
    [projection_points(i,:), distances(i)] = point3D_to_line_projection(point_cloud(i,:), line_point, line_direction);
end


figure;
hold on;
scatter3(point_cloud(:,1), point_cloud(:,2), point_cloud(:,3), 'b.');

line_end = line_point + 10 * line_direction; % Extend the line for visualization
plot3([line_point(1), line_end(1)], [line_point(2), line_end(2)], [line_point(3), line_end(3)], 'r--', 'LineWidth', 2);

% Plot the projection points
scatter3(projection_points(:,1), projection_points(:,2), projection_points(:,3), 'go', 'MarkerFaceColor', 'g');
% Connect each point to its projection point
for i = 1:num_points
    plot3([point_cloud(i,1), projection_points(i,1)], [point_cloud(i,2), projection_points(i,2)], [point_cloud(i,3), projection_points(i,3)], 'k--');
end

xlabel('X');
ylabel('Y');
zlabel('Z');
title('Point to Line Projection');
axis equal;
grid on;
view(3);
hold off;

