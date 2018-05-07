%%%%%%%%%%%%%%%%%%%% Q3.1.a and b %%%%%%%%%%%%%%%%%%%
% implement a method for estimating a homography and fundamental matrix given a set of
% corresponding points coordinates

% good background and guide on 
% https://cseweb.ucsd.edu/classes/wi07/cse252a/homography_estimation/homography_estimation.pdf
% https://en.wikipedia.org/wiki/Eight-point_algorithm
clear
clc
close all

addpath(genpath('../ransac_homography/'));
addpath(genpath('../Q2.1/'));
addpath(genpath('../Q1.2/'));

%%% from Matlab documentation: find matching points to test 
%         %homography and fundamental matx estimation %%%
% imga = imread('./tsukuba/scene1.row3.col1.ppm');
% imgb = imread('./tsukuba/scene1.row3.col2.ppm');
% imga =im2single(rgb2gray(imga)); 
% imgb =im2single(rgb2gray(imgb)); 
% % imga = imread('./HG/1x0d.jpg');
% imgb = imread('./HG/15x20d.jpg');
% imga =im2single(rgb2gray(imga)); 
% % imgb =im2single(rgb2gray(imgb));
imga = imread('./boat/img1.pgm');
imgb = imread('./boat/img2.pgm');

% %Find the corners
% pointsa = detectHarrisFeatures(imga);
% pointsb = detectHarrisFeatures(imgb);
% %Extract the neighborhood features.
% [featuresA,valid_pointsA] = extractFeatures(imga,pointsa,'Method','SURF');
% [featuresB,valid_pointsB] = extractFeatures(imgb,pointsb,'Method','SURF');
% %Match the features.
% indexPairs = matchFeatures(featuresA,featuresB);
% %Retrieve the locations of the corresponding points for each image.
% matchedPointsA = valid_pointsA(indexPairs(:,1),:);
% matchedPointsB = valid_pointsB(indexPairs(:,2),:);
%figure; showMatchedFeatures(img1,img2,matchedPoints1,matchedPoints2);

% from own harris detection
% harris_a =findInterest(imga);
% harris_b =findInterest(imgb);
% 
% match_ab = NNMatch(harris_a.Location, harris_b.Location);
% matchedPointsA = harris_a(match_ab(:,1),:);
% matchedPointsB = harris_b(match_ab(:,2),:);


%-------------HARRISDETECTOR-----------------------------------

harrisA = detectHarrisFeatures(imga, 'MinQuality', 0.05); 
harrisB = detectHarrisFeatures(imgb, 'MinQuality', 0.05);

%-------------DESCRIPTOR-----------------------------------

patchSize = 11; %should be odd
[patchA,validPointsA] = extractFeatures(imga,harrisA, 'Method', 'Block', 'BlockSize', patchSize);
[patchB,validPointsB] = extractFeatures(imgb,harrisB, 'Method', 'Block', 'BlockSize', patchSize);

%-------------NEARESTNEIGHBOURMATCHING-----------------------------------

[indexPairs, metric] = matchFeatures(patchA,patchB, 'Unique', true);
matchedPointsA = validPointsA(indexPairs(:,1),:);
matchedPointsB = validPointsB(indexPairs(:,2),:);
figure(3); 
showMatchedFeatures(imga,imgb,matchedPointsA,matchedPointsB); 


pa = round(matchedPointsA.Location(:,:));  %in (x,y) coordinates
pb = round(matchedPointsB.Location(:,:));

h = findhomography(pb,pa);  %homography matrix %p1=hp2 %algorithm seems ok but only working in certain images
f = findfundamental(pa,pb); %fundamental matrix (this is working fine)
[H, ~] = findHomography2(pa',pb'); %external function using better(?) algorithms
F = estimateFundamentalMatrix(pa,pb);
%% %%%%%%%%%%%%%%%%%% Q3.1.c %%%%%%%%%%%%%%%%%%%
% Image projection and homography accuracy
[img2_trans, ref]= projection(imga,H); %works fine when H is correct
%show projected image & original image
figure
imshowpair(imgb, imref2d(size(imgb)), img2_trans, ref); 


% Homography accuracy 
HA = errorHA(pa, pb, H); %projects arg1

%% %%%%%%%%%%%%%%%%%% Q3.1.d %%%%%%%%%%%%%%%%%%%
% matlab epipolar lines
figure
imshow(imgb,[])
hold all
plot(matchedPointsB.Location(:,1),matchedPointsB.Location(:,2),'g+')
epiLines = epipolarLine(F',matchedPointsB.Location(:,:)); % epilines are in image A
points = lineToBorderPoints(epiLines,size(imga));
line(points(:,[1,3])',points(:,[2,4])');

FA_matlab = FAerror(matchedPointsB.Location(:,:),epiLines); %matlabs error worse????

% own code (epipolar lines computed in own function)
% epipolar lines in image a
figure
imshow(imga,[])
hold all
lines = ownEpipolar(pb,f);  %pb in imgb --> epi_line in imga
points = lineToBorderPoints(lines,size(imga));
line(points(:,[1,3])',points(:,[2,4])');
plot(matchedPointsB.Location(:,1),matchedPointsB.Location(:,2),'g+')
FA = FAerror(pb,lines);

%epipolar lines in image b
figure
imshow(imgb,[])
hold all
lines = ownEpipolar(pa,f');  %pa in imga --> epi_line in imgb
points = lineToBorderPoints(lines,size(imgb));
line(points(:,[1,3])',points(:,[2,4])');
plot(matchedPointsA.Location(:,1),matchedPointsA.Location(:,2),'g+')
% fundamental matrix accuracy
FA = FAerror(pa,lines);

%% ----------------- Functions -----------------
function harrisA =findInterest(grayIA)
[yA,xA] = harrisDetect(grayIA,0.01); 
harrisA = cornerPoints([xA,yA]); 
end