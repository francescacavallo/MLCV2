% %%%%%%%%%%%%%%%%%%%% Q2.1.b %%%%%%%%%%%%%%%%%%%
clear; clc; close all
addpath ('../Q1.2');
addpath ('../Q1.3');
addpath('../ransac_homography/');
% load images
srcFiles = dir('HG/*.jpg');
[images]=loadImg(srcFiles);

imga = images{3}; graya=imga;%graya =im2single(rgb2gray(imga)); 
imgb = images{2}; grayb=imgb;%grayb =im2single(rgb2gray(imgb)); 
imgc = images{4}; grayc=imgc;%grayc =im2single(rgb2gray(imgc)); 


%% ------------------------- Manual -------------------    
% manual selection of interest points 
nIP = 10; %number of interest points 
[coord_a, coord_b, coord_c]=ManualIP(nIP,imga,imgb,imgc);

% estimate homography 
h1_manual = findhomography(coord_a',coord_b'); %own function
h2_manual = findhomography(coord_a',coord_c');
h3_manual = findhomography(coord_b',coord_c');


% visualise homographies 
%AB pair
[imgab_trans, ref]= projection(imgb,h1_manual); 
figure
imshowpair(imga, imref2d(size(imga)), imgab_trans, ref);

%AC pair
[imgac_trans, ~]= projection(imgc,h2_manual); 
figure
imshowpair(imga, imref2d(size(imga)),imgac_trans, ref);

%BC pair
[imgbc_trans, ~]= projection(imgc,h3_manual); 
figure
imshowpair(imgb, imref2d(size(imgb)),imgbc_trans, ref);

% Ha error
error1_manual = errorHA(coord_a',coord_b',h1_manual);
error2_manual = errorHA(coord_a',coord_c',h2_manual);
error3_manual = errorHA(coord_b',coord_c',h3_manual);


%% ------------------------- Automatic -------------------    
% automatic selection of interest points
harrisA =findInterest(graya);
harrisB =findInterest(grayb);
harrisC =findInterest(grayc);

% match_ab = NNMatch(harris_a.Location, harris_b.Location);
% matchedPointsA_ab = harris_a(match_ab(:,1),:);
% matchedPointsB_ab = harris_b(match_ab(:,2),:);
% 
% match_ac = NNMatch(harris_a.Location, harris_c.Location);
% matchedPointsA_ac = harris_a(match_ac(:,1),:);
% matchedPointsC_ac = harris_c(match_ac(:,2),:);
% 
% match_bc = NNMatch(harris_b.Location, harris_c.Location);
% matchedPointsB_bc = harris_a(match_bc(:,1),:);
% matchedPointsC_bc = harris_c(match_bc(:,2),:);

%harrisA = detectHarrisFeatures(graya, 'MinQuality', 0.07); 
%harrisB = detectHarrisFeatures(grayb, 'MinQuality', 0.07);
%harrisC = detectHarrisFeatures(grayc, 'MinQuality', 0.07);

patchSize = 7; %should be odd
[patchA,validPointsA] = extractFeatures(graya,harrisA, 'Method', 'Block', 'BlockSize', patchSize);
[patchB,validPointsB] = extractFeatures(grayb,harrisB, 'Method', 'Block', 'BlockSize', patchSize);
[patchC,validPointsC] = extractFeatures(grayc,harrisC, 'Method', 'Block', 'BlockSize', patchSize);

[indexPairs_ab, metric_ab] = matchFeatures(patchA,patchB, 'Unique', true);
[indexPairs_ac, metric_ac] = matchFeatures(patchA,patchC, 'Unique', true);
[indexPairs_bc, metric_bc] = matchFeatures(patchB,patchC, 'Unique', true);
%AB
matchedPointsA_ab = validPointsA(indexPairs_ab(:,1),:);
matchedPointsB_ab = validPointsB(indexPairs_ab(:,2),:);
%AC
matchedPointsA_ac = validPointsA(indexPairs_ac(:,1),:);
matchedPointsC_ac = validPointsC(indexPairs_ac(:,2),:);
%BC
matchedPointsB_bc = validPointsB(indexPairs_bc(:,1),:);
matchedPointsC_bc = validPointsC(indexPairs_bc(:,2),:);

% estimate homography
h_ab_automatic = findhomography(matchedPointsB_ab.Location,matchedPointsA_ab.Location);  %own function
h_ac_automatic = findhomography(matchedPointsC_ac.Location,matchedPointsA_ac.Location);  %own function
h_bc_automatic = findhomography(matchedPointsC_bc.Location,matchedPointsB_bc.Location);  %own function

[h_ab_automatic_ransac, inliers] = findHomography2(matchedPointsB_ab.Location',matchedPointsA_ab.Location');
[h_ac_automatic_ransac, ~] = findHomography2(matchedPointsA_ac.Location',matchedPointsC_ac.Location');
[h_bc_automatic_ransac, ~] = findHomography2(matchedPointsC_bc.Location',matchedPointsB_bc.Location');

% visualise
%AB pair
[img2_trans, ref]= projection(imgb,h_ab_automatic_ransac); 
figure
imshowpair(imga, imref2d(size(imga)), img2_trans, ref);
%AC pair
[img2_trans, ref]= projection(imga,h_ac_automatic_ransac); 
figure
imshowpair(imgc, imref2d(size(imgc)),img2_trans, ref);
%BC pair
[img2_trans, ref]= projection(imgc,h_bc_automatic_ransac); 
figure
imshowpair(imgb, imref2d(size(imgb)),img2_trans, ref);

% Ha error
error_ab_automatic = errorHA(matchedPointsA_ab.Location(inliers,:),matchedPointsB_ab.Location(inliers,:),h_ab_automatic_ransac);
error_ac_automatic = errorHA(matchedPointsA_ac.Location,matchedPointsC_ac.Location,h_ac_automatic);
error_bc_automatic = errorHA(matchedPointsA_bc.Location,matchedPointsC_bc.Location,h_ac_automatic);

%% SURF detection by Matlab
surfA = detectSURFFeatures(graya);
surfB = detectSURFFeatures(grayb);
surfC = detectSURFFeatures(grayc);

[patchA_SURF,validPointsA_SURF] = extractFeatures(graya,surfA);%, 'Method', 'Block', 'BlockSize', patchSize);
[patchB_SURF,validPointsB_SURF] = extractFeatures(grayb,surfB);%, 'Method', 'Block', 'BlockSize', patchSize);
[patchC_SURF,validPointsC_SURF] = extractFeatures(grayc,surfC);%, 'Method', 'Block', 'BlockSize', patchSize);

[indexPairs_ab_SURF, metric_ab] = matchFeatures(patchA_SURF,patchB_SURF, 'Unique', true);
[indexPairs_ac_SURF, metric_ac] = matchFeatures(patchA_SURF,patchC_SURF, 'Unique', true);
[indexPairs_bc_SURF, metric_bc] = matchFeatures(patchB_SURF,patchC_SURF, 'Unique', true);
%AB
matchedPointsA_ab_SURF = validPointsA_SURF(indexPairs_ab_SURF(:,1),:);
matchedPointsB_ab_SURF = validPointsB_SURF(indexPairs_ab_SURF(:,2),:);
matchedPointsA_ab_SURF = matchedPointsA_ab_SURF.Location;
matchedPointsB_ab_SURF = matchedPointsB_ab_SURF.Location;
%AC
matchedPointsA_ac_SURF = validPointsA_SURF(indexPairs_ac_SURF(:,1),:);
matchedPointsC_ac_SURF = validPointsC_SURF(indexPairs_ac_SURF(:,2),:);
matchedPointsA_ac_SURF = matchedPointsA_ac_SURF.Location;
matchedPointsC_ac_SURF = matchedPointsC_ac_SURF.Location;
%BC
matchedPointsB_bc_SURF = validPointsB_SURF(indexPairs_bc_SURF(:,1),:);
matchedPointsC_bc_SURF = validPointsC_SURF(indexPairs_bc_SURF(:,2),:);
matchedPointsB_bc_SURF = matchedPointsB_bc_SURF.Location;
matchedPointsC_bc_SURF = matchedPointsC_bc_SURF.Location;

% estimate homography with RANSAC
[h_ab_ransac_SURF, inliers] = findHomography2(matchedPointsB_ab_SURF',matchedPointsA_ab_SURF');
[h_ac_ransac_SURF, ~] = findHomography2(matchedPointsC_ac_SURF',matchedPointsA_ac_SURF');
[h_bc_ransac_SURF, ~] = findHomography2(matchedPointsC_bc_SURF.',matchedPointsB_bc_SURF');

% visualise
%AB pair
[img2_trans, ref]= projection(imgb,h_ab_ransac_SURF); 
figure
imshowpair(imga, imref2d(size(imga)), img2_trans, ref);
%AC pair
[img2_trans, ref]= projection(imgc,h_ac_ransac_SURF); 
figure
imshowpair(imga, imref2d(size(imga)),img2_trans, ref);
%BC pair
[img2_trans, ref]= projection(imgc,h_bc_ransac_SURF); 
figure
imshowpair(imgb, imref2d(size(imgb)),img2_trans, ref);

% Ha error
error_ab_SURF = errorHA(matchedPointsA_ab_SURF(inliers,:),matchedPointsB_ab_SURF(inliers,:),h_ab_ransac_SURF);
error_ac_SURF = errorHA(matchedPointsA_ac_SURF,matchedPointsC_ac_SURF,h_ac_ransac_SURF);
error_bc_SURF = errorHA(matchedPointsA_bc_SURF,matchedPointsC_bc_SURF,h_ac_ransac_SURF);


%% ------------------------- functions -------------------    
function [images]=loadImg(srcFiles)
for i = 1:length(srcFiles)
  directory = strcat('HG/', srcFiles(i).name);
  images{i} = imread(directory);
end
end

function harrisA =findInterest(grayIA)
[yA,xA] = harrisDetect(grayIA,0.01); 
harrisA = cornerPoints([xA,yA]); 
end