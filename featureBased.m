function [ex_mat, fail] = featureBased(Marker, Img, in_mat, fMethod, heMethod, pModel, hdMethod)
  ex_mat = eye(4); fail = false;
  
  % addpaths
  addpath('./feature_RANSAC/');
  addpath(genpath('./HE/'));
  addpath('./OPnP/');
  addpath('./DIRT_v1p2/');
  addpath('./IPPE/');
  addpath('./PHD/');
  addpath('./functions/');
  
  % preCal the nor_mat
  [h1, w1, ~] = size(Marker);
  marker_w = w1 / min(w1, h1) * 0.5;
  marker_h = h1 / min(w1, h1) * 0.5;
  nor_mat = eye(3);
  nor_mat(1, 1) = 2 * marker_w / (w1-1);
  nor_mat(1, 3) = -(w1+1)/(w1-1) * marker_w;
  nor_mat(2, 2) = -2 * marker_h / (h1-1);
  nor_mat(2, 3) = (h1+1)/(h1-1) * marker_h;
  
  % feature matching and RANSAC
  fprintf(['Feature matching using ' fMethod '...\n']);
  [P, Q, fail1] = feature_RANSAC(Marker, Img, in_mat, nor_mat, fMethod);
  if (fail1)
    [ex_mat, ~] = genPoseRandom('normal', 0, in_mat);
    fail = true;
    fprintf('  Fail!\n');
    return;
  end
  fprintf('  Success!\n');
  
  % estimate homography
  fprintf(['Homography estimation using ' heMethod '...\n']);
  [H, fail2] = HE(in_mat, nor_mat, P, Q, heMethod);
  if (fail2)
    [ex_mat, ~] = genPoseRandom('normal', 0, in_mat);
    fail = true;
    fprintf('  Fail!\n');
    return;
  end
  fprintf('  Success!\n');
  
  % DIC
  fprintf(['Dual inverse composition with photometric model ' pModel '...\n']);
  H_refined = DIC(H, Marker, Img, pModel);
  
  % get first pose
  fprintf(['Homography decomposition using ' hdMethod '...\n']);
  [ex_mat, fail3] = HD(H_refined, in_mat, nor_mat, hdMethod, w1, h1);
  assert(~fail3);
end