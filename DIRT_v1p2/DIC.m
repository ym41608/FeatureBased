% Dual inverse compositional on homography
%
% Usage:
%   H_refined = DIC(H, Marker, Img, pModel)
%
% Inputs:
%   H         = 3*3 homography
%   Marker    = RGB marker image with double type
%   Img       = RGB camera image with double type
%   pModel    = photomertic transformation model, please reference Groupwise Geometric and Photometric Direct Image Registration, TPAMI, 2008
%
% Outputs:
%   H_refined = refined 3*3 homography

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% Dec, 2015
function H_refined = DIC(H, Marker, Img, pModel)
  H_refined = eye(3);
  
  % change to [y x 1] mode
  H = [H(:,2) H(:,1) H(:,3)];
	H = [H(2,:); H(1,:); H(3,:)];

  % mask the region of interest
	M = DIRT_MaskEdges(rgb2gray(Marker), 2);
  
  % precompute parameters
  if (strcmp(pModel, 'no_poptim'))
    pre = DIRT_Precomputation(Marker, 'ROI', DIRT_Mask2ROI(M), 'no_poptim', 'verb', 0);
  else
    pre = DIRT_Precomputation(Marker, 'ROI', DIRT_Mask2ROI(M), 'pmodel', pModel, 'verb', 0);
  end
  
  % DIC
	reg = DIRT_Registration(Img, pre, 'ginit', H, 'verb', 0);
  fprintf('  finish in %d iterations\n', reg.it);
  if (reg.err || (reg.e_roi_init < reg.e_roi))
    H_refined = H;
  else
    H_refined =reg.g;
  end
  
  % change back to [x y 1] mode
  H_refined = [H_refined(:,2) H_refined(:,1) H_refined(:,3)];
	H_refined = [H_refined(2,:); H_refined(1,:); H_refined(3,:)];
end