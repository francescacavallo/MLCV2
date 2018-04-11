function [ desc ] = getDescriptors( img, points, bins )
%DESCRIBE Describes patches given by 2xN matrix of points
%   Returns an MxN matrix of descriptors for the points

% Uses each point as the top (16, 16) point of a 32x32 patch
% Takes a histogram of each patch and adds to the matrix

    % Create output matrix
    N = size(points, 2);
    M = bins;
    desc = zeros(M, N);

    % Patch size
    patchSize = 32;
    offset = floor(patchSize/2);
    offset_tau = floor((patchSize - 1)/2);
    
    % Pad image
    img = padarray(img, [offset, offset], 'replicate');
    % Offset points (1, 1) should become (17, 17);
    points = points + offset;
    
    % Get patch represented by pixel
    for index=1:N
        h = points(2, index);
        w = points(1, index);
        patch = img(h-offset_tau:h+offset, w-offset_tau:w+offset);

        % Get the histogram of the patch
        descriptor = imhist(patch, M);

        % Add the descriptor to the output matrix
        desc(:, index) = descriptor;

    end
    
end