%%%%% Q2.1.c %%%%
%setup
clear; clc; close all
addpath ('../Q1.2');
addpath ('../Q1.3');
addpath('../ransac_homography/');
% load images
srcFiles = dir('HG/*.jpg');
[images]=loadImg(srcFiles);
imga = images{1}; graya =im2single(rgb2gray(imga)); 
imgb = images{2}; grayb =im2single(rgb2gray(imgb)); 

% Use only AB pair cause it's the one working fine with Harris+NNM
[matchedPointsA, matchedPointsB]=findMatch(graya,grayb); 

error=zeros(size(matchedPointsA,1),1);
error_ransac=zeros(size(matchedPointsA,1),1);
n=size(matchedPointsA,1)-1;

for corres=4:size(matchedPointsA,1)
    % randomly chose corres points from 1 to 324 (=total matching points)
    match_idx = round(rand(corres,1)*n+1);
    %match_idx=corres;
    
    %estimate homography
    h = findhomography(matchedPointsB(match_idx,:),matchedPointsA(match_idx,:));  %own function
    [h_ransac, inliers] = findHomography2(matchedPointsA(match_idx,:)',matchedPointsB(match_idx,:)');

    %calculate HA
    error_ransac(corres)=nan;
    error(corres)=nan;
    if inliers>0
        error_ransac(corres)=errorHA(matchedPointsA(inliers,:),matchedPointsB(inliers,:),h_ransac);
        error(corres)=errorHA(matchedPointsA(match_idx,:),matchedPointsB(match_idx,:),h);
    end
end

figure
stem(error)
figure
plot(error_ransac, 'lineWidth',2)
xlabel('Number of corresponding points')
ylabel('HA error')

%show outlier
%outlier_a=find(matchedPointsA(1)==805);
%outlier_b=find(matchedPointsB(1)==597.1);
%showMatchedFeatures(imga,imgb,matchedPointsA(outlier_a,:),matchedPointsB(outlier_b,:)); %Matlab function
showMatchedFeatures(imga,imgb,matchedPointsA,matchedPointsB); %Matlab function


%% ---------- Functions ---------
function [images]=loadImg(srcFiles)
for i = 1:length(srcFiles)
  directory = strcat('HG/', srcFiles(i).name);
  images{i} = imread(directory);
end
end

function [matchedPointsA, matchedPointsB]=findMatch(graya,grayb)
%harris_a =findInterest(graya);
%harris_b =findInterest(grayb);
harris_a = detectHarrisFeatures(graya, 'MinQuality', 0.07); 
harris_b = detectHarrisFeatures(grayb, 'MinQuality', 0.07);

patchSize = 11;
[patchA,validPointsA] = extractFeatures(graya,harris_a, 'Method', 'Block', 'BlockSize', patchSize);
[patchB,validPointsB] = extractFeatures(grayb,harris_b, 'Method', 'Block', 'BlockSize', patchSize);

[indexPairs_ab, metric_ab] = matchFeatures(patchA,patchB, 'Unique', true);
matchedPointsA = validPointsA(indexPairs_ab(:,1),:);
matchedPointsB = validPointsB(indexPairs_ab(:,2),:);

%match_ab = NNMatch(harris_a.Location, harris_b.Location);
%matchedPointsA = harris_a(match_ab(:,1),:);
%matchedPointsB = harris_b(match_ab(:,2),:);
matchedPointsA = matchedPointsA.Location;
matchedPointsB = matchedPointsB.Location;
end

function harrisA =findInterest(grayIA)
[yA,xA] = harrisDetect(grayIA,0.07); 
harrisA = cornerPoints([xA,yA]); 
end