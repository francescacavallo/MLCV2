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

%%% from Matlab documentation: find matching points to test 
        %homography and fundamental matx estimation %%%
img1 = imread('./pics/img1.pgm');
img2 = imread('./pics/img2.pgm');
%Find the corners
points1 = detectHarrisFeatures(img1);
points2 = detectHarrisFeatures(img2);
%Extract the neighborhood features.
[features1,valid_points1] = extractFeatures(img1,points1,'Method','SURF');
[features2,valid_points2] = extractFeatures(img2,points2,'Method','SURF');
%Match the features.
indexPairs = matchFeatures(features1,features2);
%Retrieve the locations of the corresponding points for each image.
matchedPoints1 = valid_points1(indexPairs(:,1),:);
matchedPoints2 = valid_points2(indexPairs(:,2),:);
%figure; showMatchedFeatures(img1,img2,matchedPoints1,matchedPoints2);

p1 = round(matchedPoints1.Location(:,:));  %in (x,y) coordinates
p2 = round(matchedPoints2.Location(:,:));

h = findhomography(p1,p2);  %homography matrix %p1=hp2 %algorithm seems ok but only working in certain images
f = findfundamental(p2,p1); %fundamental matrix (this is working fine)
[H, ~] = findHomography2(p2',p1'); %external function using better(?) algorithms

%% %%%%%%%%%%%%%%%%%% Q3.1.c %%%%%%%%%%%%%%%%%%%
% Image projection and homography accuracy
[img2_trans, ref]= projection(img2,H); %works fine when H is correct
%show projected image
figure
imshow(img2_trans)

% Homography accuracy 
HA = errorHA(p2, p1, H); %projects arg1

%% %%%%%%%%%%%%%%%%%% Q3.1.d %%%%%%%%%%%%%%%%%%%
% matlab epipolar lines
figure
imshow(img2,[])
hold all
plot(matchedPoints1.Location(:,1),matchedPoints1.Location(:,2),'go')
epiLines = epipolarLine(f',matchedPoints2.Location(:,:));
points = lineToBorderPoints(epiLines,size(img2));
line(points(:,[1,3])',points(:,[2,4])');
FA_matlab = FAerror(matchedPoints2.Location(:,:),epiLines); %matlabs error worse????

% own code (epipolar lines computed in own function)
figure
imshow(img2,[])
hold all
lines = ownEpipolar(p2,f);  %p2 in img2 --> epi_line in img1
points = lineToBorderPoints(lines,size(img2));
line(points(:,[1,3])',points(:,[2,4])');

% Fundamental matrix accuracy
FA = FAerror(p2,lines);
