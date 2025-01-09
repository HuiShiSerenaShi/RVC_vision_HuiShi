clear all;
clc;
close all;

% Read the RGB and range image
% fileName = 'sofa1';
fileName = 'refrigerator';
img = imread([fileName, '.jpg']);
depth = imread([fileName,'.png']);
figure(1)
imshow(img);
figure(2)
imagesc(depth);

% Set camera parameters 
fu = 525;           % Focal length (x-direction)
fv = 525;           % Focal length (y-direction)
uo = 319.5;          % Principal point(x-direction)
vo = 239.5;          % Principal point(y-direction)

cloud_3D = [];
triangles = [];

X=zeros(size(depth));
Y=zeros(size(depth));
TRUE=zeros(size(depth));
vertex_ind = zeros(size(depth));
ind=0;

% Iterate through each pixel in the depth image
for i = 2:size(depth, 1)
    for j = 2:size(depth, 2)
        % Get the depth value 
        z = double(depth(i, j));
        
        if z == 0
            continue;
        end
        
       % Compute the 3D coordinates
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
figure(3)
plot3(cloud_3D(:,1),cloud_3D(:,2),cloud_3D(:,3), 'b.');
xlabel('X');
ylabel('Y');
zlabel('Z');

% X84 rule to calculate the threshold of edge of triangles
K_X84=5.2;
res=zeros((size(depth, 1)*size(depth, 2))*8,1);
ind=0;
for i=3:size(depth, 1)
    for j=3:size(depth, 2)
        if(TRUE(i,j)==1)
            v1=[X(i,j) Y(i,j) double(depth(i, j))];
            for h=0:2
            for k=0:2
                if(TRUE(i-h,j-k)==1)
                    ind=ind+1;
                    v2=[X(i-h,j-k) Y(i-h,j-k) double(depth(i-h,j-k))];
                    res(ind)=norm(v1-v2);
                end
            end
            end
        end
    end
end
res=res(1:ind);
MED_x84 =median(res);
MAD_x84=K_X84*median(abs(res-MED_x84));
TX_X84=MED_x84+MAD_x84;


% Iterate through each pixel and generate triangles
% [ * * ~
%   * * ~
%   ~ ~ ~ ] , the * are the 4 vertexes of 2 triangles
waitbar_ = waitbar(0, 'generating mesh ...');
for i = 2:size(depth, 1)
    waitbar(i/(size(depth, 1)-2),waitbar_)
    for j = 2:size(depth, 2)

        z = double(depth(i, j));
        if z == 0
            continue;
        end

        % Get index of 4 vertexes
        center_px = vertex_ind(i,j);
        upper_px =vertex_ind(i-1,j);
        left_px=vertex_ind(i,j-1);
        upper_left_px=vertex_ind(i-1,j-1);
        
        % check if the vertexes of the lower triangle exist
        if upper_px == 0 || left_px == 0
            continue 
        end

        % generate the lower triangle if the 3 edges are all within the
        % threshold obtained from the X84 rule
        if norm(cloud_3D(center_px,:)-cloud_3D(upper_px,:)) <= TX_X84 && norm(cloud_3D(center_px,:)-cloud_3D(left_px,:)) <= TX_X84 && norm(cloud_3D(upper_px,:)-cloud_3D(left_px,:))<= TX_X84
            triangles = [triangles; [center_px upper_px left_px]];
        end

        % check if the vertex of the upper triangle exist
        if upper_left_px == 0
            continue 
        end

        % generate the upper triangle if the 3 edges are all within the
        % threshold obtained from the X84 rule
        if norm(cloud_3D(upper_left_px,:)-cloud_3D(upper_px,:)) <= TX_X84 && norm(cloud_3D(upper_left_px,:)-cloud_3D(left_px,:))<= TX_X84 && norm(cloud_3D(upper_px,:)-cloud_3D(left_px,:))<= TX_X84
            triangles = [triangles; [upper_px left_px upper_left_px ]];
        end
    end
end

% figure(4)
% trisurf(triangles, cloud_3D(:, 1), cloud_3D(:, 2), cloud_3D(:, 3));
% hold on
% axis equal

% export cloud_3D and triangles as ply file which can be visualized in Meshlab
nvertex=size(cloud_3D,1);
exportMeshToPly(cloud_3D,triangles,ones(nvertex,3), fileName);