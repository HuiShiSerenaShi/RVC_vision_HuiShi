clear all;
clc;
close all;

I = imread('eight.tif');
figure(1);
imshow(I);

% Create a disk-shaped structuring element
se = strel('disk',10);
% se = strel('disk',15);
% se = strel('disk',20);
% se = strel('disk',25);

% remove small noise
open_img = imopen(I,se);
figure(2);
imshow(open_img);

% Enhance contrast
contrast_img = imadjust(open_img);
figure(3);
imshow(contrast_img)

% Convert to binary image
bw = not(imbinarize(contrast_img));
bw = bwareaopen(bw,30);
figure(4);
imshow(bw)

% Find connected components
cc = bwconncomp(bw,4);
cc.NumObjects

% Extract a specific coin, here is the 4th
% coins = false(size(bw));
% coins(cc.PixelIdxList{4}) = true;
% figure(5);
% imshow(coins);
% title('4th coin')

% Label connected components
labeled = labelmatrix(cc);
whos labeled
RGB_label = label2rgb(labeled,'spring','c','shuffle');
figure(6);
imshow(RGB_label)

% Get region properties
coin_data = regionprops(cc,'basic');
coin_areas = [coin_data.Area];
% Extract the largest coin
[max_area, idx] = max(coin_areas);
coins = false(size(bw));
coins(cc.PixelIdxList{idx}) = true;
figure(7);
imshow(coins);
title('largest coin')

% Plot histogram of coin areas
figure(8);
histogram(coin_areas)
title('Histogram of Coin Area')

