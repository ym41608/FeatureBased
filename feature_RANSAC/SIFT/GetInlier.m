function [tar_coor, scr_coor, failed] = GetInlier( tarframes, scrframes, matches, thres, iter, P)
% Get the inliers of the matched feature pairs using RANSAC
%
% Usage:
%   [tar_coor, scr_coor] = GetInlier( tarframes, scrframes, matches, thres, iter, P)
%
% Inputs:
%   tarframes = 2*n feature coordinates of target frame 
%   scrframes = 2*n feature coordinates of screen (camera) frame 
%   matches   = indices of matched features
%   thres     = distance threshold of the transformed correspondence
%   iter      = number of iterations
%   P         = intrinsic matrix
%
% Outputs:
%   tar_coor  = inlier coordinates of target image
%   scr_coor  = inlier coordinates of screen image
%   failed    = whether the RANSAC is failed

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% June 1, 2014

thres_sq = thres*thres;
num_of_matched = size(matches, 2);
max_inliers = 0;
A = ones(3, num_of_matched);

% --- Find the initial T
for i = 1:iter
    i_T = GenRand(tarframes, scrframes, matches, 4);
    A(1:2, :) = tarframes(:,matches(1,:));
    B = i_T*A;
    project_coor = [B(1,:)./B(3,:); B(2,:)./B(3,:)];
    correct_coor = scrframes(:,matches(2,:));
    distance_sq = sum((project_coor-correct_coor).^2);
    
    % Vote and choose the best parameters
    num = 0;
    for i=1:num_of_matched
        if distance_sq(i) < thres_sq
            num = num +1;
        end
    end
    if num > max_inliers
        max_inliers = num;
        T = i_T;  
    end  
end

fprintf('  Maximum number of inliers: %d\n', max_inliers);

if (max_inliers == 0)
    failed = 1;
    tar_coor = [];
    scr_coor = [];
    return;
end


% --- Refine the obtained T from RANSAC
inliers = zeros(2, max_inliers);
A(1:2, :) = tarframes(:,matches(1,:));
B = T*A;
project_coor = [B(1,:)./B(3,:); B(2,:)./B(3,:)];
correct_coor = scrframes(:,matches(2,:));
distance_sq = sum((project_coor-correct_coor).^2);
num = 1;
for i=1:num_of_matched
    if distance_sq(i) < thres_sq
        inliers(:,num) = matches(:, i);
        num = num+1;
    end
end

tar_coor = [tarframes(1:2, inliers(1,:)); zeros(1,max_inliers)];
scr_coor = [(scrframes(1,inliers(2,:))-P(1,3))/P(1,1);(scrframes(2,inliers(2,:))-P(2,3))/P(2,2)];
failed = 0;

    
    








