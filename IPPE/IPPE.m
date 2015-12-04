% Get two extrinsic matrixes and its reprojection errors through IPPE(IJCV 2014)
%
% Usage:
%   [ex_mat_set reprojErr_set] = IPPE(K, P, Q, H)
%
% Inputs:
%   K             = 3*3 camera intrinsic matrix
%   P             = 2*n or 3*n matrix holding the model points in world coordinates. Points are defined on the plane z=0. 
%                   If P is 3xn then the third row must be all-zeros.
%   Q             = 2*n matrix holding the points in the camera image. These are defined in pixels.
%   H             = 3*3 homography matrix
%
% Outputs:
%   ex_mat_set    = 4*4*2 matrix, which is two 4*4 extrinsic matrix
%   reprojErr_set = corresponding reprojection errors

% Most of the codes are implementated by Toby Collins in his work "Infinitesimal Plane-based Pose Estimation", IJCV 2014.
% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Oct, 2015
function [ex_mat_set reprojErr_set] = IPPE(K, P, Q, H)
  
  %n is the number of points:
  n = size(P,2);
  
  %zero-center model points:
  U = P(1:2, :);
  Pbar = [0; 0; 0];
  
  %transform correspondences to normalised pixel coordinates:
  Kinv = inv(K);
  Q_n = Kinv(1:2,:)*[Q;ones(1,n)];
  
  %Compute the Jacobian J of the homography at (0,0):
  H = H./H(3,3);
  J(1,1) = H(1,1)-H(3,1)*H(1,3);
  J(1,2) = H(1,2)-H(3,2)*H(1,3);
  J(2,1) = H(2,1)-H(3,1)*H(2,3);
  J(2,2) = H(2,2)-H(3,2)*H(2,3);
  
  %Compute the two rotation solutions:
  v = [H(1,3);H(2,3)];
  [R1,R2] = IPPE_dec(v,J);
  
  %Compute the two translation solutions:
  t1_ = estT(R1,U,Q_n);
  t2_ = estT(R2,U,Q_n);
  t1 = [R1,t1_]*[-Pbar;1];
  t2 = [R2,t2_]*[-Pbar;1];
  
  %Sort the solutions:
  [R1,R2,t1,t2,reprojErr1,reprojErr2] = sortSolutions(R1,R2,t1,t2,K,P,Q);
  
  %Assign the output
  ex_mat_set = repmat(eye(4), [1 1 2]);
  ex_mat_set(1:3, 1:4, 1) = [R1, t1];
  ex_mat_set(1:3, 1:4, 2) = [R2, t2];
  reprojErr_set = [0 0];%[reprojErr1 reprojErr2];
end



function t = estT(R,psPlane,qq)
  %Computes the least squares estimate of translation given the rotation solution.
  if size(psPlane,1)==2
      psPlane(3,:) =0;
  end
  %qq = homoMult(Kinv,pp')';
  Ps = R*psPlane;
  
  numPts = size(psPlane,2);
  Ax = zeros(numPts,3);
  bx = zeros(numPts,1);
  
  Ay = zeros(numPts,3);
  by = zeros(numPts,1);
  
  
  
  Ax(:,1) = 1;
  Ax(:,3) = -qq(1,:);
  bx(:) = qq(1,:).*Ps(3,:) -  Ps(1,:);
  
  Ay(:,2) = 1;
  Ay(:,3) = -qq(2,:);
  by(:) = qq(2,:).*Ps(3,:) -  Ps(2,:);
  
  A = [Ax;Ay];
  b = [bx;by];
  
  AtA = A'*A;
  Atb = A'*b;
  
  Ainv = IPPE_inv33(AtA);
  t = Ainv*Atb;
end