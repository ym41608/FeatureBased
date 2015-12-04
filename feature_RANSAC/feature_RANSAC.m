% feature matching and RANSAC
%
% Usage:
%   [P Q fail] = feature_RANSAC(Marker, Img, in_mat)
%
% Inputs:
%   Marker = RGB marker image with double type
%   Img = RGB camera image with double type
%   in_mat = 4*4 intrinsic matrix
%   nor_mat = 3*3 matrix which normalize pixel coordinates to marker coordinates
%   fMethod = feature matching method, 'ASIFT' and 'SIFT'
%
% Outputs:
%   P         = 3*n inlier coordinates of marker image (z = 0)
%   Q         = 2*n inlier coordinates of camera image (normalized coordinate)
%   fail      = indicates the matching is fail or not

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Dec, 2015
function [P, Q, fail] = feature_RANSAC(Marker, Img, in_mat, nor_mat, fMethod)
  P = []; Q = []; fail = false;
  warning('off', 'all');
  Marker = rgb2gray(Marker);
  Img = rgb2gray(Img);
  switch fMethod
    case 'ASIFT'
      cd 'feature_RANSAC/ASIFT';
      [P, Q, fail] = ASIFT_RANSAC(Marker, Img, in_mat, nor_mat);
      cd '../..';
    case 'SIFT'
      cd 'feature_RANSAC/SIFT';
      [P, Q, fail] = SIFT_RANSAC(Marker, Img, in_mat, nor_mat);
      cd '../..';
  end
end