function [mean_dist] = FAerror(points,epilines)
dist = zeros(size(points,1),1);
for i=1:size(points,1)
    x0 = points(i,1);
    y0 = points(i,2);
    a = epilines(i,1);
    b = epilines(i,2);
    c = epilines(i,3);
    dist(i)=abs(a*x0+b*y0+c)/(sqrt(a^2+b^2));
end
mean_dist = mean(dist);

end

