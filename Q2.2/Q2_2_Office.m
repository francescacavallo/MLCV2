close all; clear;

%Office dataset
IA = imread('Office\left_14.jpg');
IB = imread('Office\right_14.jpg');
L16 = imread('Office\left_16.jpg');
R16 = imread('Office\right_16.jpg');
IA = imresize(IA, 0.1);
IB = imresize(IB, 0.1);
L16 = imresize(L16, 0.1);
R16 = imresize(R16, 0.1);
figure(1); imshow(IA); title('Image A');
figure(2); imshow(IB); title('Image B');
grayIA = im2single(rgb2gray(IA)); 
grayIB = im2single(rgb2gray(IB));
grayIAFocal = im2single(rgb2gray(L16)); 
grayIBFocal = im2single(rgb2gray(R16));

%Tsukuba dataset
%{
srcFiles = dir('Tsukuba\*.ppm'); 
for i = 1:length(srcFiles)
  directory = strcat('Tsukuba\', srcFiles(i).name);
  images{i} = imread(directory);
end
IA = images{1}; 
IB = images{2}; 
figure(1); imshow(IA); title('Image A');
figure(2); imshow(IB); title('Image B');
grayIA = im2single(rgb2gray(IA)); 
grayIB = im2single(rgb2gray(IB));
%}

%My harris detect
%{ 
[yA,xA] = harrisDetect(grayIA,0.01); 
[yB,xB] = harrisDetect(grayIB,0.01);
harrisA = cornerPoints([xA,yA]); 
harrisB = cornerPoints([xB,yB]);
%}

minQuality = 0.01; %lower -> more matches, less quality
harrisA = detectHarrisFeatures(grayIA, 'MinQuality', minQuality); 
harrisB = detectHarrisFeatures(grayIB, 'MinQuality', minQuality);

figure(1); imshow(IA); hold on; 
figure(2); imshow(IB); hold on; 

%plot(harrisA);
%plot(harrisB);

patchSize = 19; %higher -> more reliable matches
[patchA,validPointsA] = extractFeatures(grayIA,harrisA, 'Method', 'Block', 'BlockSize', patchSize);
[patchB,validPointsB] = extractFeatures(grayIB,harrisB, 'Method', 'Block', 'BlockSize', patchSize);
%descriptorsA = hist(patchA',255)'; descriptorsB = hist(patchB',255)';

maxRatio = 0.7 ; %higher -> more matches (0 to 1)
matchThreshold = 10.0 ; %higher -> more matches (0 to 10)
[indexPairs, metric] = matchFeatures(patchA,patchB, 'Unique', true, 'MaxRatio', maxRatio, 'MatchThreshold', matchThreshold);
matchedPointsA = validPointsA(indexPairs(:,1),:);
matchedPointsB = validPointsB(indexPairs(:,2),:);
figure(9); showMatchedFeatures(IA,IB,matchedPointsA,matchedPointsB); 

%My NN matching
%{
correspondance = NNMatch(descriptorsA,descriptorsB);
matchedPointsA = validPointsA(correspondance(:,1),:); %doing the inlier thing
matchedPointsB = validPointsB(correspondance(:,2),:);
figure(9);
showMatchedFeatures(IA,IB,matchedPointsA,matchedPointsB); %Matlab function
%}

%-------------STARTQ2.2-----------------------------------

%A)

%My fundamental matrix
%F = findfundamental(matchedPointsA.Location,matchedPointsB.Location);

[F, inliers] = estimateFundamentalMatrix(matchedPointsA.Location,matchedPointsB.Location);
matchedPointsA = matchedPointsA(inliers,:);
matchedPointsB = matchedPointsB(inliers,:);

epilinesA = epipolarLine(F',matchedPointsB.Location); 
epilinesB = epipolarLine(F,matchedPointsA.Location);

FaccA = FAerror(matchedPointsA.Location,epilinesA);
FaccB = FAerror(matchedPointsB.Location,epilinesB);

%B)

[isInA, epipoleA] = isEpipoleInImage(F , size(grayIA));
[isInB, epipoleB] = isEpipoleInImage(F', size(grayIB));

borderptsA = lineToBorderPoints(epilinesA,size(grayIA));
borderptsB = lineToBorderPoints(epilinesB,size(grayIB));

figure(1); line(borderptsA(:,[1,3])',borderptsA(:,[2,4])'); 
hold on; plot(matchedPointsA.Location(:,1),matchedPointsA.Location(:,2),'go');
hold on; plot(epipoleA(1), epipoleA(2), '-o', 'MarkerFaceColor',[1 0 0], 'MarkerSize',10);
figure(2); line(borderptsB(:,[1,3])',borderptsB(:,[2,4])'); 
hold on; plot(matchedPointsB.Location(:,1),matchedPointsB.Location(:,2),'go');
hold on; plot(epipoleB(1), epipoleB(2), '-o', 'MarkerFaceColor',[1 0 0], 'MarkerSize',10);

blockSize = 5;
disparityRange = [0 16];
disparityMap = disparity(grayIA,grayIB, 'BlockSize', blockSize,'DisparityRange', disparityRange);

figure(3); imshow(disparityMap, disparityRange);
colormap(gca,jet); colorbar;

%C)

%Tsukuba
%{
focal = 615; %Tsukuba 615 pixels
baseline = 0.10; %Tsukuba 10cm between 2 neighbour images
%}

% Sensor width Nikkor 1 J2 model -> 13.2mm
% Image width Nikkor 1 J2 model -> 3872
% F_pixel = F_meter * Img_width_pixel / Sensor_width_meter
focal_14 = 0.014 * 3872 /0.0132; %Office dataset 14mm -> 407 pixels 
focal_16 = 0.016 * 3872 /0.0132; %Office dataset 16mm -> 466 pixels  
focal = focal_14;
baseline = 0.20; %Office dataset 20cm

depthMap = focal*baseline./disparityMap;

Upper = 15; %set manually
Lower = 4; %set manually
depthMap(find(depthMap(:) > Upper)) = Upper;
depthMap(find(depthMap(:) < Lower)) = Lower;

figure(4); surf(depthMap,'edgecolor','interp','LineStyle','none','FaceColor','interp');
zlim auto; caxis auto; colormap('jet'); colorbar;

%D)

%Change the focal length by 2mm, repeat Q2.2.c and compare.
disparityMapFocal = disparity(grayIAFocal,grayIBFocal,'BlockSize',5,'DisparityRange',[0 16]);
focalnew = focal_16; %Put relevant parameter per dataset
depthMapFocal = focalnew*baseline./disparityMapFocal;

Upper = 15; %set manually
Lower = 4; %set manually
depthMapFocal(find(depthMapFocal(:) > Upper)) = Upper;
depthMapFocal(find(depthMapFocal(:) < Lower)) = Lower;

figure(5); 
surf(depthMapFocal,'edgecolor','interp','LineStyle','none','FaceColor','interp');
zlim auto; caxis auto; colormap('jet'); colorbar;

%Add small random noise (e.g. Gaussian with max 2 pixel), repeat Q2.2.c and compare. 
disparityMapNoisy = imnoise(disparityMap./16,'gaussian', 0, 0.00000676).*16;

% 0_16 range is mapped to 0_256 pixel values
% 1/8=0.125 is mapped to 2 pixels
% after compression, 0.125/16 = 0.0078
% Variation caused by noise is +- 0.0078 of original value
% 3 sigma is 99.7% of distribution
% so 0.0078 = 3*sigma -> sigma = 0.0026 -> var = 0.00000676

figure(6); imshow(disparityMapNoisy,[0 16]);
colormap(gca,jet); colorbar;

depthMapNoisy = focal*baseline./disparityMapNoisy;

%depthMapNoisy(find(depthMapNoisy(:) > Upper)) = Upper;
%depthMapNoisy(find(depthMapNoisy(:) < Lower)) = Lower;

figure(7); surf(depthMapNoisy,'edgecolor','interp','LineStyle','none','FaceColor','interp');
zlim auto; caxis auto; colormap('jet'); colorbar;

%E)

%Present stereo rectified pair of your images

%Done above already
%{
[F, inliers] = estimateFundamentalMatrix(matchedPointsA.Location,matchedPointsB.Location);
matchedPointsA = matchedPointsA(inliers,:);
matchedPointsB = matchedPointsB(inliers,:);
%}

[t1, t2] = estimateUncalibratedRectification(F, matchedPointsA.Location, matchedPointsB.Location, size(IA));

tform1 = projective2d(t1);
tform2 = projective2d(t2);

[IARect, IBRect] = rectifyStereoImages(IA, IB, tform1, tform2, 'OutputView', 'full');

figure(8); imshow(stereoAnaglyph(IARect, IBRect));
%title('Rectified Stereo Images (Red - Left Image, Cyan - Right Image)');



