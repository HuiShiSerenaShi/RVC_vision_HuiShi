% compute cloud from range image
function [cloud_3D, X, Y] = range_to_cloud(depth, fu, fv, uo, vo)

    cloud_3D = [];
    X=zeros(size(depth));
    Y=zeros(size(depth));
    TRUE=zeros(size(depth));
    vertex_ind = zeros(size(depth));
    ind=0;
    
    for i = 1:size(depth, 1)
        for j = 1:size(depth, 2)

            z = double(depth(i, j));
     
            if z == 0
                continue;
            end
            
            x = (j - uo) * z / fu; % column is x
            y = (i - vo) * z / fv; 
    
            X(i,j)=x;
            Y(i,j)=y;
            TRUE(i,j)=1;
            ind=ind+1;
            vertex_ind(i,j)=ind;
            cloud_3D = [cloud_3D; [x, y, z]];
        end
    end
end