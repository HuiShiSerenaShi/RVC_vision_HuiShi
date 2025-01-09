% Angle between two lines
% The angle θ between two lines l1 and l2 is recovered from their inner product:
% θ = arcos(⟨l1, l2⟩)

function angle = angle_between_lines(line1_direction, line2_direction)
    
    line1_direction = line1_direction / norm(line1_direction);
    line2_direction = line2_direction / norm(line2_direction);
   
    dot_product = dot(line1_direction, line2_direction);
    angle = acos(dot_product);
end
