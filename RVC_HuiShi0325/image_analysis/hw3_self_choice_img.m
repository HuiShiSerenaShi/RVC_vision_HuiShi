% Differentiate the pink rectangular toy and three yellow ducks in the image, and sort the ducks by size.
% 1. Thresholding is based on color.
% 2. Then distinguish between the pink rectangular toy and the ducks based on circularity.
% 3. Next, sort the ducks by area size.

clear all;
clc;
close all;

% Read the image
I = imread('toys1.jpg');
figure;
imshow(I);
title('Original Image');

% Extract light pink and yellow objects，thresholding based on color.
pink_mask = (I(:,:,1) > 200) & (I(:,:,2) < 240) & (I(:,:,3) < 240); % Threshold for light pink
yellow_mask = (I(:,:,1) > 200) & (I(:,:,2) > 200) & (I(:,:,3) < 100); % Threshold for yellow
% Combine the two masks to obtain light pink and yellow objects
target_mask = pink_mask | yellow_mask;

figure;
imshow(target_mask);
title('Binarized Image');

% Fill holes in the binarized Image
target_mask  = imfill(target_mask , 'holes');
figure;
imshow(target_mask);
title('fill hole');

% Dilate the image to connect nearby regions
target_mask = imdilate(target_mask,strel('disk', 10));
figure;
imshow(target_mask);
title('Dilation Image');
% Erode the image to refine the shapes
target_mask = imerode(target_mask, strel('disk', 10)); 
figure;
imshow(target_mask);
title('Erosion Image');

% Calculate connected components
cc = bwconncomp(target_mask, 4);

% Display the labeled connected components image
labeled = labelmatrix(cc);
RGB_label = label2rgb(labeled, 'spring', 'c', 'shuffle');
figure;
imshow(RGB_label);
title('Labeled Connected Components');

fprintf('Number of connected components: %d\n', cc.NumObjects);
% Get measurements of connected components
stats = regionprops(cc, 'Centroid', 'Area', 'Circularity');

% Sort connected components by area
[~, idx] = sort([stats.Area], 'descend');

% Mark the centroids of objects and add labels
figure;
imshow(I);
hold on;
duck_count = 0;

for i = 1:cc.NumObjects
    centroid = stats(idx(i)).Centroid;
    area = stats(idx(i)).Area;
    circularity = stats(idx(i)).Circularity

    % Differentiate between ducks and the rectangular toy based on circularity
    % sort the ducks by area size.
    if circularity > 0.85  
        duck_count = duck_count + 1;
        
        if duck_count == 1
            text(centroid(1), centroid(2), '  Biggest Duck', 'Color', 'g', 'FontSize', 12);
        elseif duck_count == 2
            text(centroid(1), centroid(2), '  Second Big Duck', 'Color', 'g', 'FontSize', 12);
        elseif duck_count == 3
            text(centroid(1), centroid(2), '  Smallest Duck', 'Color', 'g', 'FontSize', 12);
        end
    else
        text(centroid(1), centroid(2), '  Pink Toy', 'Color', 'b', 'FontSize', 12);
    end
    
    plot(centroid(1), centroid(2), 'r.', 'MarkerSize', 20);
end

hold off;
title('Objects with Labels');
