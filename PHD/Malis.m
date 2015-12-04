% Get two extrinsic matrix and nits reprojection errors through perspective homography decomposition method proposed in 2007 by Malis
%
% Usage:
%   [ex_mat_set N_set] = Malis(K, P, Q, H)
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

% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Oct, 2015
function [ex_mat_set reprojErr_set] = Malis(K, P, Q, H)
  
	% get normalized homography
	sValue = svd(H);
	H = H / sValue(2);
	
	% Homography decomposition
	[ex_mat_set N_set] = decomposeHomography(H);
  
  % The one with N ~ [0 0 1] is the first pose
  %[~, setId] = sort(N_set' * [0 0 1]');
  %ex_mat_set(:, :, 1) = ex_mat_set(:, :, setId(2));
  %ex_mat_set(:, :, 2) = ex_mat_set(:, :, setId(1));
  %reprojErr_set = [0, 0];
  
  % sorting
  [R1,R2,t1,t2,reprojErr1,reprojErr2] = sortSolutions(ex_mat_set(1:3, 1:3, 1), ex_mat_set(1:3, 1:3, 2), ...
                                                      ex_mat_set(1:3, 4, 1), ex_mat_set(1:3, 4, 2), K, P, Q);
  
  % assignment
  ex_mat_set(1:3, :, 1) = [R1, t1];
  ex_mat_set(1:3, :, 2) = [R2, t2];
  reprojErr_set = [reprojErr1, reprojErr2];
end


function [ex_mat_set, N_set] = decomposeHomography(H)
	% S matrix
	S = H' * H - eye(3);
	% element for vector N
	M11 =  -det(S([2 3], [2 3]));
	M22 =  -det(S([1 3], [1 3]));
	M33 =  -det(S([1 2], [1 2]));
	e12 = (-det(S([2 3], [1 3])) >= 0) * 2 - 1;
	e23 = (-det(S([1 3], [1 2])) >= 0) * 2 - 1;
	e13 = (-det(S([2 3], [1 2])) >= 0) * 2 - 1;
	rtM11 = sqrt(M11);
	rtM22 = sqrt(M22);
	rtM33 = sqrt(M33);

	% find max |Sii|
	[~, idx] = max([diag(abs(S))]);

	% vector Na, Nb
	Na = zeros(1, 3); Nb = zeros(1, 3);
	switch (idx)
		case 1
			Na = [S(1,1); S(1,2)+rtM33; S(1,3)+e23*rtM22];
			Nb = [S(1,1); S(1,2)-rtM33; S(1,3)-e23*rtM22];
		case 2
			Na = [S(1,2)+rtM33; S(2,2); S(2,3)-e13*rtM11];
			Nb = [S(1,2)-rtM33; S(2,2); S(2,3)+e13*rtM11];
		case 3
			Na = [S(1,3)+e12*rtM22; S(2,3)+rtM11; S(3,3)];
			Nb = [S(1,3)-e12*rtM22; S(2,3)-rtM11; S(3,3)];
	end
	Na = Na / norm(Na);
	Nb = Nb / norm(Nb);
	N_set = [Na Nb];
	
	% element for star_t
	S_trace = trace(S);
	v = 2 * sqrt(1 + S_trace - M11 - M22 - M33);
	es = (S(idx, idx) >= 0);
	rho = sqrt(2 + S_trace + v);
	2 + S_trace - v;
	te_norm = sqrt(2 + S_trace - v);
	te_norm_half = te_norm * 0.5;
	
	% star_t
	star_ta = te_norm_half * (es*rho*Nb - te_norm*Na);
	star_tb = te_norm_half * (es*rho*Na - te_norm*Nb);

	% R1 t1
	R1 = H * (eye(3) - (2/v) * star_ta * Na');
	t1 = R1 * star_ta;
	
	% R2 t2
	R2 = H * (eye(3) - (2/v) * star_tb * Nb');
	t2 = R2 * star_tb;
	
	% transform to target coordinates from imaginative ref camera
	tgc = [1 0 0 0; 0 1 0 0; 0 0 1 1; 0 0 0 1];
	
	% 2 poses
  ex_mat_set = repmat(eye(4), [1 1 2]);
  ex_mat_set(1:3,:,1) = [R1, t1] * tgc;
	ex_mat_set(1:3,:,2) = [R2, t2] * tgc;
  ex_mat_set(1:3,3,1) = ex_mat_set(1:3,3,1) * ((ex_mat_set(3,3,1)>=0)*2 - 1);
  ex_mat_set(1:3,3,2) = ex_mat_set(1:3,3,2) * ((ex_mat_set(3,3,2)>=0)*2 - 1);
	%ex_mat_set(:,3,:) = -ex_mat_set(:,3,:);
end