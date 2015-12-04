% Homography estimation
%
% Usage:
%   [H fail] = HE(in_mat, nor_mat, P, Q, heMethod)
%
% Inputs:
%   in_mat    = 4*4 intrinsic matrix
%   nor_mat   = 3*3 matrix which normalize pixel coordinates to marker coordinates
%   P         = 3*n inlier coordinates of marker image (z = 0)
%   Q         = 2*n inlier coordinates of camera image (normalized coordinate)
%   heMethod  = homography estimation method, 'Harker', 'DLT' and 'OPnP'
%
% Outputs:
%   H         = 3*3 homography
%   fail      = indicates the matching is fail or not

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Dec, 2015
function [H fail] = HE(in_mat, nor_mat, P, Q, heMethod)
  H = eye(3); fail = false;
  K = in_mat(1:3,1:3);
  
  if (strcmp(heMethod, 'OPnP'))
    % OPnP
    [R, t] = OPnP(P, Q);
    if (size(R,1) ~= 3 || size(R,2) ~= 3)
      fail = true;
      return;
    else
      ex_mat = eye(3);
      ex_mat(:, 1:2) = R(:, 1:2, 1);
      ex_mat(:, 3) = t(:, 1);
      H = K * ex_mat * nor_mat
    end
  else
    %n is the number of points:
    n = size(P,2);
    
    %compute the model-to-image homography:
    switch heMethod
        case 'DLT'
            H = homography2d([P(1:2, :);ones(1,n)],[Q;ones(1,n)]);
        case 'Harker'
            H = homographyHarker([P(1:2, :);ones(1,n)],[Q;ones(1,n)]);
    end
    H = K * H * nor_mat;
  end
end