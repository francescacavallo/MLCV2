function [lines] = ownEpipolar(points,F)
% input:    points(n,2)= matching points in image
%           F(3,3): fundamental matrix between img1 and img2
%output:    lines(a,b,c): ax+by+c=0 --> one for each point
lines=zeros(size(points,1),3);
for i=1:size(points,1)
    lines(i,:)=F*[points(i,:)';1];
end

end

