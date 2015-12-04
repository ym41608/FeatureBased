% Homography decomposition
%
% Usage:
%   [ex_mat fail3] = HD(H, in_mat, nor_mat, hdMethod, w1, h1)
%
% Inputs:
%   H         = 3*3 homography
%   in_mat    = 4*4 intrinsic matrix
%   nor_mat   = 3*3 matrix which normalize pixel coordinates to marker coordinates
%   hdMethod  = homography decomposition method, 'Malis', 'IPPE' and 'OPnP'
%   w1, h1    = width and height of marker image
%
% Outputs:
%   ex_mat    = 4*4 extrinsic matrix
%   fail      = indicates the decomposition is fail or not

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Dec, 2015
function [ex_mat fail] = HD(H, in_mat, nor_mat, hdMethod, w1, h1)
  ex_mat = eye(4); fail = false;
  
  % get four point
  P = [1,w1,w1,1;1,1,h1,h1;1,1,1,1];
  Q = H * P;
	P = nor_mat * P;
	P(3,:) = 0;
	Q(1,:) = Q(1,:)./Q(3,:);
	Q(2,:) = Q(2,:)./Q(3,:);
	Q_n = [(Q(1,:)-in_mat(1,3))/in_mat(1,1); (Q(2,:)-in_mat(2,3))/in_mat(2,2)];
  
  % get 3*3 intrinsic matrix
  K = in_mat(1:3, 1:3);
  
  % get normalized homography
	H = inv(K) * H * inv(nor_mat);
  
  switch(hdMethod)
    case 'Malis'
      [ex_mat_set, ~] = Malis(K, P, Q(1:2, :), H);
    case 'IPPE'
      [ex_mat_set, ~] = IPPE(K, P, Q(1:2, :), H);
    case 'OPnP'
      [RR, tt, ~, ~] = OPnP(P, Q_n);
      if (size(RR,3) ~= size(tt,2))
        fail = true;
        return;
      else
        ex_mat_set = repmat(eye(4), [1 1 2]);
        ex_mat_set(1:3, 1:3, 1)= RR(:,:,1); 
        ex_mat_set(1:3, 4, 1) = tt(:,1);
        ex_mat_set(1:3, 1:3, 2)= RR(:,:,2); 
        ex_mat_set(1:3, 4, 2) = tt(:,2);
      end
  end
	ex_mat = ex_mat_set(:, :, 1);
  ex_mat(:, 3) = -ex_mat(:, 3);
end