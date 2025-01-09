% Point to plane projection
% Given a generic point p ∈ R and given that d(p) = ⟨p, n⟩ −d, we can define the point to plane projection as:
% p′ = p − n · d(p)

function projected_point = point3D_to_plane_projection(point, plane_normal, plane_d)
    distance_to_plane =  point3D_to_plane_distance(point, plane_normal, plane_d);
    projected_point = point - plane_normal * distance_to_plane;
end
