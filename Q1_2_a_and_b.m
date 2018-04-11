close all; clear;

%Load images into struct

srcFiles = dir('Tsukuba\*.ppm');
for i = 1:length(srcFiles)
  directory = strcat('Tsukuba\', srcFiles(i).name);
  images{i} = imread(directory);
end

%figure(1); imshow(images{1}); title('Image 1');
%figure(2); imshow(images{2}); title('Image 2');

%Load 2 images in single precision gray
IA = images{1}; grayIA = im2single(rgb2gray(IA)); 
IB = images{2}; grayIB = im2single(rgb2gray(IB));

%figure(1); imshow(grayIA); title('Image 1');
%figure(2); imshow(grayIB); title('Image 2');

%grayIA = imresize(grayIA,2); grayIB = imresize(grayIB,2);
%grayIA = imrotate(grayIA,30); grayIB = imrotate(grayIB,30);

%-------------HARRISDETECTOR-----------------------------------

%harrisA: cornerPoint object

[yA,xA] = harrisDetect(grayIA,0.01); 
[yB,xB] = harrisDetect(grayIB,0.01);

harrisA = cornerPoints([xA,yA]); 
harrisB = cornerPoints([xB,yB]);

%Matlab implementation
%harrisA = detectHarrisFeatures(grayIA); harrisB = detectHarrisFeatures(grayIB);

%figure(1); imshow(grayIA); hold on; plot(harrisA);

%-------------DESCRIPTOR-----------------------------------

% patchA: descriptors in row, length (31*31=961) in column.
% validPointsA: (x,y) of point at the origin of descriptor.
% descriptorsA: colour histogram so 255 gray values

%descriptorsB = getDescriptors( grayIA, harrisA, 255);
%descriptorsB = getDescriptors( grayIB,harrisB, 255);

%Matlab implementation
patchSize = 39 %should be odd
[patchA,validPointsA] = extractFeatures(grayIA,harrisA, 'Method', 'Block', 'BlockSize', patchSize);
[patchB,validPointsB] = extractFeatures(grayIB,harrisB, 'Method', 'Block', 'BlockSize', patchSize);
descriptorsA = hist(patchA',255)'; descriptorsB = hist(patchB',255)';

%-------------NEARESTNEIGHBOURMATCHING-----------------------------------

correspondance = NNMatch(descriptorsA,descriptorsB);
matchedPoints1 = validPointsA(correspondance(:,1),:);
matchedPoints2 = validPointsB(correspondance(:,2),:);
showMatchedFeatures(IA,IB,matchedPoints1,matchedPoints2); %Matlab function

%Matlab implementation
%?