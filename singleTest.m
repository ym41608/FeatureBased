clc; clear all; close all;
Marker = im2double(imread('imgs/Isetta.png'));
I = im2double(imread('imgs/32.png'));
[height, width, ~] = size(I);
focal_length = norm([height, width]);
in_mat = [focal_length,0,width/2,0;0,-focal_length,height/2,0;0,0,1,0;0,0,0,1];

fMethod = 'ASIFT'; %'SIFT' ASIFT'
heMethod = 'DLT';%'Harker'; %'DLT' 'OPnP'
pModel = 'SingleGainAndBias';%'no_poptim' , 'SingleGainAndBias' 'MultipleGainsAndBiases' 'Affine'
hdMethod = 'IPPE';%'Malis' 'IPPE' 'OPnP'

[ex_mat, fail] = featureBased(Marker, I, in_mat, fMethod, heMethod, pModel, hdMethod);

f = figure('Position', [150 150 1280 720]);
[corner_x, corner_y] = draw_coordinate(ex_mat, in_mat);
imagesc(I); 
truesize; axis off; hold on;
plot([corner_x(1);corner_x(2)], [corner_y(1);corner_y(2)], 'r', 'LineWidth', 5);
plot([corner_x(1);corner_x(3)], [corner_y(1);corner_y(3)], 'g', 'LineWidth', 5);
plot([corner_x(1);corner_x(4)], [corner_y(1);corner_y(4)], 'b', 'LineWidth', 5);