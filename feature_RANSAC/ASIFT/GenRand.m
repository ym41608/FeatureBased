function T = GenRand( tarframes, scrframes, corres)
% Get random samples and calculate the perspective transformation matrix
%
% Usage:
%   T = GenRand( tarframes, scrframes, corres)
%
% Inputs:
%   tarframes = 2*n feature coordinates of target frame 
%   scrframes = 2*n feature coordinates of screen (camera) frame 
%   corres    = point-pairs (correspondances) to get the perspective transformation matrix)
%
% Outputs:
%   T         = transformation matrix

%% Programmed by:
% Lab421
% Graduate Institute of Electronics Engineering, National Taiwan University, Taipei, Taiwan
% June 1, 2014

num_of_matched = size(tarframes, 2);

%rand('seed',99)
if corres==num_of_matched
    C = 1:corres;
else
    C = round((num_of_matched-1)*(rand(1, corres))+1);
end

A = zeros(2*corres, 8);
B = zeros(2*corres, 1);

% --- Construct A & B
bi = 1; ei = corres;
A(bi:ei,1:2) = tarframes(:,C(:))';
A(bi:ei,3)   = ones(corres, 1);
A(bi:ei,7)   = -1*tarframes(1, C(:)).*scrframes(1, C(:));
A(bi:ei,8)   = -1*tarframes(2, C(:)).*scrframes(1, C(:));
bi = corres+1; ei = corres+corres;
A(bi:ei,4:5) = tarframes(:, C(:))';
A(bi:ei,6)   = ones(corres, 1);
A(bi:ei,7)   = -1*tarframes(1, C(:)).*scrframes(2, C(:));
A(bi:ei,8)   = -1*tarframes(2, C(:)).*scrframes(2, C(:));
B(:) = [scrframes(1, C(:)), scrframes(2, C(:))];

% --- Solve AX = B
if corres==4
    X = A\B;
else
    X = (A'*A)\(A'*B);
end

T = [X(1:3)'; X(4:6)'; X(7:8)' 1];
