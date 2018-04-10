function [err] = HA(imga,imgb)
% homography accuracy between transformed imgb and original imga

for r=1:size(imga,1) %ueclidean distance
    for c=1:size(imga,2)
    distance(i) =sqrt((imgb(i,2)-imga(i,2))^2+(imgb(i,1)-imga(i,1))^2);
end

err = mean(distance);

end

