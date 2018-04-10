function [img_transformed, ref] = projection(imgb, H)
%project point from image B to image A given homography between them

width=size(imgb,2); %columns
height=size(imgb,1); %rows
projected = zeros(height * width,3);

%transformation
i=1;
for r=1:height
    for c=1:width
        coord = H*[c;r;1];       
        projected(i,:) = [coord(1)/coord(3),coord(2)/coord(3),double(imgb(r,c))]; %note:change uint8 to double       
        i=i+1;
    end
end

% build actual displayable image
XWorldLimits = [min(projected(:,1)), max(projected(:,1))];
YWorldLimits = [min(projected(:,2)), max(projected(:,2))];
tot_cols = round(max(projected(:,1))-min(projected(:,1)));
tot_rows = round(max(projected(:,2))-min(projected(:,1)));

img_transformed = uint8(zeros(tot_rows, tot_cols));
ref = imref2d([tot_rows, tot_cols]);
ref.XWorldLimits = [XWorldLimits(1), XWorldLimits(2)];
ref.YWorldLimits = [YWorldLimits(1), YWorldLimits(2)];

% Populate
for i = 1:size(projected, 1)
    X = projected(i, 1);
    Y = projected(i, 2);
    colour = projected(i, 3);
    X = round(X - XWorldLimits(1)) + 1;
    Y = round(Y - YWorldLimits(1)) + 1;
    img_transformed(Y, X) = colour;
    %distance = 
    %error = img_transformed
end

end
    %distance(i) =sqrt((imgb(i,2)-imga(i,2))^2+(imgb(i,1)-imga(i,1))^2);


