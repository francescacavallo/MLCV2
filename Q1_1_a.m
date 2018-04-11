close all; clear;

%Load images into struct

srcFiles = dir('Tsukuba\*.ppm');
for i = 1:length(srcFiles)
  directory = strcat('Tsukuba\', srcFiles(i).name);
  images{i} = imread(directory);
end

figure(1); imshow(images{1}); title('Image 1');
figure(2); imshow(images{2}); title('Image 2');

%Select K interest points

K = 10;
for i = 1:K
    figure(1); [x_one(i),y_one(i)] = ginput(1);
    hold all; scatter(x_one(i),y_one(i),'o','filled');
    
    figure(2); [x_two(i),y_two(i)] = ginput(1);
    hold all; scatter(x_two(i),y_two(i),'o','filled');
end
