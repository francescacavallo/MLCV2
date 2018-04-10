function [f] = findfundamental(pa,pb)
% find fundamental matrix: solve x'Fx in F
% or solve Af=0

%adjust size: pa and pb have x,y in the cols and pairs in the rows
if size(pa,2)>size(pa,1)&&size(pa,1)>1
    pa=pa';
end
if size(pb,2)>size(pb,1)&&size(pa,1)>1
    pb=pb';
end

% build A matrix from x'Fx
A=zeros(size(pb,1),9);
for i=1:size(pa,1)
    xa = pa(i, 1); ya = pa(i, 2);
    xb = pb(i, 1); yb = pb(i, 2);
    A(i,:) = [xa*xb, xa*yb, xa, xb*ya, ya*yb, ya, xb, yb, 1];
end

[~,~,V]=svd(A'*A);

l = V(:,9)/V(9,9);

f = reshape(l,3,3)'; %make h into matrix

end

