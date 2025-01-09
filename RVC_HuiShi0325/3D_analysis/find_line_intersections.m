% find line intersections
% check if the lines are parallel, if not, compute intersections
function [intersection_points, valid_pairs] = find_line_intersections(lines)
    num_lines = size(lines, 1);
    intersection_points = [];
    valid_pairs = [];
    
    if num_lines < 2
        disp('At least two lines are required for intersection calculation.');
        return;
    end

    for i = 1:num_lines-1
        for j = i+1:num_lines
            line1 = lines(i, :);
            line2 = lines(j, :);
            
            % calculate angle_between_lines
            angle = angle_between_lines(line1(4:6), line2(4:6));
            
            % check if the lines are parallel, parallel lines the angle is
            % close to 0. if not, compute intersections
            if angle > deg2rad(30) && angle < deg2rad(120)
                intersection_point = line3D_intersection(line1(1:3), line1(4:6), line2(1:3), line2(4:6));
                
                intersection_points = [intersection_points; intersection_point];
                valid_pairs = [valid_pairs; i, j];
            end
        end
    end
end
