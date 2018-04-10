function [ average_distance ] = errorHA( p1, p2, h )
%ERRORHA Calculates average distance between two sets of corresponding
% points. Translates points1 by transformation first. 
%   Input: p1: nx2 
%          p2: nx2
n=size(p1,1);    
    % Transform points1
    proj_p1 = zeros(size(p1));
    for i = 1:n
        temp = h * [p1(i,:), 1]';
        proj_p1(i,:) = temp(1:2)/temp(3);
    end

    % Euclidean distance between each pair of points.
    diff = proj_p1(1:n,:) - p2(1:n,:);
    diff = diff .^2;
    diff = sum(diff,2);
    diff = sqrt(diff);
    
    % Average it out for HA value.
    average_distance = mean(diff);

end

