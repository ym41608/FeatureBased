% ASIFT matching and RANSAC
%
% Usage:
%   [P Q fail] = ASIFT_RANSAC(Marker, Img, in_mat)
%
% Inputs:
%   Marker = grayscale marker image with double type
%   Img = grayscale camera image with double type
%   in_mat = 4*4 intrinsic matrix
%   nor_mat = 3*3 matrix which normalize pixel coordinates to marker coordinates
%
% Outputs:
%   P         = 3*n inlier coordinates of marker image (z = 0)
%   Q         = 2*n inlier coordinates of camera image (normalized coordinate)
%   fail      = indicates the matching is fail or not

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Dec, 2015
function [P, Q, fail] = ASIFT_RANSAC(Marker, Img, in_mat, nor_mat)
  P = []; Q = []; fail = false;
  % add path
  addpath('./ASIFT');
  
  % for demo_ASIFT use
  imwrite(Marker, 'tempI1.png');
  imwrite(Img, 'tempI2.png');
  imgOutVert = 'imgOutVert.png';
  imgOutHori = 'imgOutHori.png';
  matchings = 'matchings.txt';
  keys1 = 'keys1.txt';
  keys2 = 'keys2.txt';
  flag_resize = 0;
  
  % ASIFT
  [frames1, frames2] = ASIFT('tempI1.png', 'tempI2.png', imgOutVert, imgOutHori, matchings, keys1, keys2, flag_resize);
  
  % Normalize pixel coordinates and change direction of y axis.
  frames1 = nor_mat * [frames1; ones(1, size(frames1,2))];%[((frames1(1,:)-w1/2)./(w1/2))*marker_w; -((frames1(2,:)-h1/2)/(h1/2)*marker_h)];
  
  if (size(frames1, 2) == 0 || size(frames2, 2) == 0)
    fail = true;
  else
    % RANSAC
    [P, Q] = GetInlier(frames1(1:2, :), frames2(1:2, :), 10, 1000, in_mat);
    fail = false;
  end
end



