function H = homographySVD( xy,xyb )

%Compute Homography
for j = 1:size(xy,2)

    A(2*j-1,:) = [xy(1,j), xy(2,j), 1, 0, 0,0,-xy(1,j)*xyb(1,j), -xyb(1,j)*xy(2,j), -xyb(1,j)];
    A(2*j,:) = [0,0,0, xy(1,j), xy(2,j),1, -xy(1,j)*xyb(2,j),-xyb(2,j)*xy(2,j), -xyb(2,j)];

end

[u,d,v] = svd(A'*A);
 h = v(:,9);
 H = reshape(h,3,3)';
 H = H/H(end);

end

