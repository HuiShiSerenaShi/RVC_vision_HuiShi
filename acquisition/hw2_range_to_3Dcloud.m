clear all;
clc;
close all;

% Read the RGB and range image
% img = imread('sofa1.jpg');
% depth = imread('sofa1.png');
img = imread('refrigerator.jpg');
depth = imread('refrigerator.png');
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
% Iterate through each pixel in the depth image
for i = 1:size(depth, 1)
    for j = 1:size(depth, 2)

        % Get the depth value 
        z = double(depth(i, j));
        
        if z == 0
            continue;
        end
        
        % Compute the 3D coordinates 
        x = (j - uo) * z / fu; % column is x
        y = (i - vo) * z / fv; % 
       
        cloud_3D = [cloud_3D; [x, y, z]];
    end
end

figure(3)
plot3(cloud_3D(:,1),cloud_3D(:,2),cloud_3D(:,3), 'b.');
xlabel('X');
ylabel('Y');
zlabel('Z');