% Point to line projection
% Given a generic point q ∈ R3 , its projection to the line l(p, v) is given by:
% q′ = (⟨(q − p), v⟩) · v. Its distance to the line is d = ∥q − q′∥.

function [projection_point, distance] = point3D_to_line_projection(point_cloud, line_point, line_direction)

    line_direction = line_direction / norm(line_direction); 
    
    % Compute the vector from the 3D point to the line
    vec_to_line = point_cloud - line_point;
    
    % Compute the projection point
    projection_scalar = dot(vec_to_line, line_direction);
    projection_point = line_point + projection_scalar * line_direction;
    
    distance = vecnorm(point_cloud - projection_point, 2, 2);
end
