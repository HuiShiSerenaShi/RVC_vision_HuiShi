% Given two lines l1 = (p1, v1) and l2 = (p2, v2) on the 3D space, their
% intersection is given by the midpoint of the minimum segment that connects
% the two lines. This segment is identified by points pa ∈ l1 and pb ∈ l2.
% pa = p1 + µa · v1, pb = p2 + µb · v2, more details please see the notes from moodle

function intersection_point = line3D_intersection(p1, v1, p2, v2)
   
    alpha = dot(p1 - p2, v1);
    beta = dot(v1, v1);
    gamma = dot(v2, v1);
    psi = dot(p1 - p2, v2);
    k = dot(v2, v2);

    denominator = beta * k - gamma * gamma;
    mu_a = (psi * gamma - alpha * k) / denominator;
    mu_b = (psi + mu_a * gamma) / k;

    pa = p1 + mu_a * v1;
    pb = p2 + mu_b * v2;
    intersection_point = (pa + pb) / 2;
end
