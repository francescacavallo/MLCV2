%%%%%%%%%%%%%%%%%%%% Q2.1 %%%%%%%%%%%%%%%%%%%
clear 
clc
close all

addpath ('../Q1.2');
addpath ('../Q1.3');
addpath('../ransac_homography/');

%Load images into struct
srcFiles = dir ('HG/*.jpg');
for i = 1:length(srcFiles)
  directory = strcat('HG/', srcFiles(i).name);
  images{i} = imread(directory);
end

% ------------ Compare original size and reduced by 2 -----------
%images
IA = images{3}; grayIA =im2single(rgb2gray(IA)); 
IA_reduced2 = imresize(IA, 0.5); grayIA_reduced2 = imresize(grayIA, 0.5);

% Find interest points using method from Q1
harrisA =findInterest(grayIA);
harrisA_reduced2 =findInterest(grayIA_reduced2);
plotIP(IA, harrisA)
plotIP(IA_reduced2, harrisA_reduced2)

% Match points based on interest points and not features
match = NNMatch(harrisA.Location, 2*harrisA_reduced2.Location);
matchedPointsA = harrisA(match(:,1),:);
matchedPointsA2 = harrisA_reduced2(match(:,2),:);

% plot interest points from two images on the same original image
figure
imshow(IA);
hold on;
scatter(matchedPointsA.Location(:, 1), matchedPointsA.Location(:, 2), 50, 'x', 'MarkerEdgeColor', 'green');
hold on;
scatter(2.*matchedPointsA2.Location(:, 1), 2.*matchedPointsA2.Location(:, 2), 50, 'x', 'MarkerEdgeColor', 'red');
% overlap images showing matching points
figure
showMatchedFeatures(IA,IA_reduced2,matchedPointsA,matchedPointsA2); %Matlab function

%compute HA error
error2 = HA(matchedPointsA.Location, 2*matchedPointsA2.Location);

%% --------------------- Reduce image by 3 -------------------
IA_reduced3 = imresize(IA, 1/3); grayIA_reduced3 = imresize(grayIA, 1/3);

% Find interest points using method from Q1
harrisA_reduced3 =findInterest(grayIA_reduced3);
plotIP(IA, harrisA)
plotIP(IA_reduced3, harrisA_reduced3)

% Match points based on interest points and not features
match = NNMatch(harrisA.Location, 3*harrisA_reduced3.Location);
matchedPointsA = harrisA(match(:,1),:);
matchedPointsA3 = harrisA_reduced3(match(:,2),:);

% plot interest points from two images on the same original image
figure
imshow(IA);
hold on;
scatter(matchedPointsA.Location(:, 1), matchedPointsA.Location(:, 2), 50, 'x', 'MarkerEdgeColor', 'green');
hold on;
scatter(3.*matchedPointsA3.Location(:, 1), 3.*matchedPointsA3.Location(:, 2), 50, 'x', 'MarkerEdgeColor', 'red');
% overlap images & matching points
figure
showMatchedFeatures(IA,IA_reduced3,matchedPointsA,matchedPointsA3); 

%compute HA error
error3 = HA(matchedPointsA.Location, 3*matchedPointsA3.Location);






    
%------------------------- functions -------------------    
function harrisA =findInterest(grayIA)
[yA,xA] = harrisDetect(grayIA,0.01); harrisA = cornerPoints([xA,yA]); 
end

function []=plotIP(IA, harrisA)
figure
imshow(IA);
hold on;
scatter(harrisA.Location(:, 1), harrisA.Location(:, 2), 50, 'x', 'MarkerEdgeColor', 'green');
end

function average_distance = HA(p1,p2)
diff = p1(1:end,:) - p2(1:end,:);
diff = diff .^2;
diff = sum(diff,2);
diff = sqrt(diff);
average_distance = mean(diff);
end
