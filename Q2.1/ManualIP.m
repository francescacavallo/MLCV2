function [coord_a,coord_b,coord_c] = ManualIP(K,img1,img2,img3)
% manually detect interest points
% figure 1, figure 2, K= number of interest points
figure(1); imshow(img1); title('Image 1');
figure(2); imshow(img2); title('Image 2');
figure(3); imshow(img3); title('Image 3');

for i = 1:K
    figure(1); [x_one(i),y_one(i)] = ginput(1);
    hold all; scatter(x_one(i),y_one(i),'o','filled');
    
    figure(2); [x_two(i),y_two(i)] = ginput(1);
    hold all; scatter(x_two(i),y_two(i),'o','filled');
    
    figure(3); [x_three(i),y_three(i)] = ginput(1);
    hold all; scatter(x_three(i),y_three(i),'o','filled');
    
    coord_a = [x_one; y_one];
    coord_b = [x_two; y_two];
    coord_c = [x_three; y_three];
end



end

