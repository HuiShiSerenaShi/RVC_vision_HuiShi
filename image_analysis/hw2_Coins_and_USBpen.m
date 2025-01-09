clear all;
clc;
close all;

I = imread('mycoins.jpg');
figure(1);
imshow(I);

% Create a disk-shaped structuring element
se = strel('disk',25);
% se = strel('disk',15);
% se = strel('disk',20);

% remove small noise
open_img = imopen(I,se);
figure(2);
imshow(open_img);

% Enhance contrast
open_img_gray = rgb2gray(open_img);
contrast_img = imadjust(open_img_gray);
figure(3);
imshow(contrast_img)

% Convert to binary image
bw = imbinarize(contrast_img);
bw = bwareaopen(bw,20);
bw = imdilate(bw,se);
figure(4);
imshow(bw)

% Find connected components 
cc = bwconncomp(bw,4);
cc.NumObjects

% Label connected components
labeled = labelmatrix(cc);
whos labeled
RGB_label = label2rgb(labeled,'spring','c','shuffle');
figure(5);
imshow(RGB_label)

% Get region properties
stats = regionprops(cc, 'Centroid','MajorAxisLength','MinorAxisLength','Circularity');

% Set circularity threshold to distinguish between circles and rectangles
threshold_circle = 0.9;
figure(6);
imshow(I);
hold on;
% Process each region
for k = 1:numel(stats)
    centroid = stats(k).Centroid;
    majorAxisLength = stats(k).MajorAxisLength;
    minorAxisLength = stats(k).MinorAxisLength;
    circularity = stats(k).Circularity;
    
   % Determine shape based on circularity
    if circularity > threshold_circle
      
        % Check if it's a big or small circle
        is_big_circle = false;
        for j = 1:numel(stats)
            if j ~= k
               % Compare sizes with other circle
                other_majorAxisLength = stats(j).MajorAxisLength;
                other_minorAxisLength = stats(j).MinorAxisLength;
                if majorAxisLength > other_majorAxisLength && minorAxisLength > other_minorAxisLength
                    is_big_circle = true;
                    break;
                end
            end
        end
        
        if is_big_circle
            text_str = '  big coin';
            text_color = 'r';
        else
            text_str = '  small coin';
            text_color = 'b';
        end
    else
        % If not a circle, consider it a rectangle (USB disk)
        text_str = '  USB disk';
        text_color = 'g';
    end

    plot(centroid(1), centroid(2), 'r.', 'MarkerSize', 15);
    text(centroid(1), centroid(2), text_str, 'Color', text_color);
    if circularity > threshold_circle
        viscircles(centroid, max(majorAxisLength, minorAxisLength)/2, 'EdgeColor', 'r');
    end
end
hold off;