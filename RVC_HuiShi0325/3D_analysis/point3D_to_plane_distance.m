% Point to plane distance
% Given a generic point p ∈ R, its distance d(p) to the plane (n, d) is given by:
% d(p) = ⟨p, n⟩ − d

function distance = point3D_to_plane_distance(point, plane_normal, plane_d)

    distance = dot(point, plane_normal) - plane_d;
end
