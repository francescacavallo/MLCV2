function [r,c] = harrisDetect(I, quality)

%Calculate derivatives of image
x_mask = [-1 0 1]; %3*3 window
y_mask = x_mask';
A = imfilter(I,x_mask,'replicate','same','conv');
B = imfilter(I,y_mask,'replicate','same','conv');

%Remove Uneceesary gradients at border
A = A(2:end-1,2:end-1);
B = B(2:end-1,2:end-1);

%figure(3); imshow(A); title('Image 3');
%figure(4); imshow(B); title('Image 4');

%Square of derivatives
C = A .* B;
A = A .* A;
B = B .* B;

%figure(3); imshow(A); title('Image 3');
%figure(4); imshow(B); title('Image 4');
%figure(5); imshow(C); title('Image 5');

%Applying blur
blur =  [0.03 0.105 0.222 0.286 0.222 0.105 0.03];

A = imfilter(A,blur,'replicate','full','conv');
B = imfilter(B,blur,'replicate','full','conv');
C = imfilter(C,blur,'replicate','full','conv');

%figure(3); imshow(A); title('Image 3');
%figure(4); imshow(B); title('Image 4');
%figure(5); imshow(C); title('Image 5');

% Clip to image size
removed = max(0, (size(blur,1)-1) / 2 - 1);
A = A(removed+1:end-removed,removed+1:end-removed);
B = B(removed+1:end-removed,removed+1:end-removed);
C = C(removed+1:end-removed,removed+1:end-removed);

%figure(3); imshow(A); title('Image 3');
%figure(4); imshow(B); title('Image 4');
%figure(5); imshow(C); title('Image 5');

%Calculate Harris
%k = 0.04; 
k = 0.000;
R = (A .* B) - (C .^ 2) - k * ( A + B ) .^ 2; %the metric
maxharr = quality * max(R(:)); %calculate threshold as 1% of biggest R response

%figure(6); imshow(R*100); title('Image 6');

location = (R > imdilate(R, [1 1 1; 1 0 1; 1 1 1])); % Nonmax Suppression
location(R < maxharr) = 0; %no detection when under threshold

%figure(7); imshow(location); title('Image 7');

% Exclude points on the border
location(1, :) = 0;
location(end, :) = 0;
location(:, 1) = 0;
location(:, end) = 0;
    
%Find Location of interest points
[r,c] = find(location);

end