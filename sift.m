clear all;
close all;

imga = imread('Data/0031.jpg');
imgb = imread('Data/0001.jpg');

la = single(rgb2gray(imga));
lb = single(rgb2gray(imgb));
[fa, da] = vl_sift(la, 'PeakThresh', 0.5, 'EdgeThresh', 10);
[fb, db] = vl_sift(lb, 'PeakThresh', 0.5, 'EdgeThresh', 10, 'Levels', 3);

%perm = randperm(size(fa,2));
%sel = perm(1:50);
figure; 

subplot(1,2,1)
image(imga);
hold on;
x = fa(1,:); y = fa(2,:);

plot(x,y, 'o');
subplot(1,2,2)
imshow(imgb);
x = fb(1,:); y = fb(2,:);
hold on;
plot(x,y, 'o');



[matches, scores] = vl_ubcmatch(da, db);

perm = randperm(size(matches,2));
sel = perm(1:10);
% for i=1:10
%     figure;
%     subplot(1,2,1)
%     image(imga);
%     hold on;
%     x = fa(1,matches(1,sel(i))); y = fa(2,matches(1,sel(i)));
% 
%     plot(x,y, 'o');
%     subplot(1,2,2)
%     imshow(imgb);
%     x = fb(1,matches(2,sel(i))); y = fb(2,matches(2,sel(i)));
%     hold on;
%     plot(x,y, 'o');
% end

