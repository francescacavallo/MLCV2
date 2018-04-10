function [h] = findhomography(pa,pb)
% estimate homography matrix wiht least squares minimisation
% input: pa(nx2)=points for image 1, pb=points for image 2
% output: h=homography matrix
% pa=hpb;

%adjust size: pa and pb have x,y in the cols and pairs in the rows
if size(pa,2)>size(pa,1)&& size(pa,1)>1
    pa=pa';
end
if size(pb,2)>size(pb,1)&&size(pa,1)>1
    pb=pb';
end

%create A matrix with all matched points from x'=Hx
A=zeros(2*size(pb,1),9);
j=0;
for i=1:size(pa,1) 
    xa = pa(i, 1); ya = pa(i, 2);
    xb = pb(i, 1); yb = pb(i, 2);
    j=j+1;
    A(j,:)= [xb yb 1 0 0 0 -xa*xb -xa*yb -xa];
    j=j+1;
    A(j,:) = [0 0 0 xb yb 1 -xb*ya -ya*yb -ya];
end

[~,S,V]=svd(A'*A); %SVD of A'A square matrix to solve Ah=0

l = V(:,9); %least squares solution is last col of V (normalised)

h = reshape(l,3,3)'; %make h into matrix
h=h./h(3,3); %normalise h
end

