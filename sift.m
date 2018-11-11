clear all;
close all;


addpath(genpath('vlfeat-0.9.21'))
data_dir = 'Data2';

data_dir = 'Data';
list = dir(data_dir);
for idx =1:32
    path = fullfile(data_dir, list(idx+2).name);
    imga = imread(path);

    [h, w, c] = size(imga);
    imga = imresize(imga, [h/2, w/2]);
    la = single(rgb2gray(imga));
    [fa1, da1] = vl_sift(la, 'PeakThresh', 0, 'EdgeThresh', 10);

    
    % lab_he = rgb2lab(imga);
    % ab = lab_he(:,:,2:3);
    % nrows = size(ab,1);
    % ncols = size(ab,2);
    % ab = reshape(ab,nrows*ncols,2);
    % 
    % nColors = 4;
    % % repeat the clustering 3 times to avoid local minima
    % [cluster_idx, cluster_center] = kmeans(ab,nColors,'distance','sqEuclidean', ...
    %                                       'Replicates',3);
    % pixel_labels = reshape(cluster_idx,nrows,ncols);
    % imshow(pixel_labels,[]), title('image labeled by cluster index');

    figure(1);
    subplot(3,1,1)
    image(imga);
    hold on;
    x = fa1(1,:); y = fa1(2,:);
    plot(x,y, 'o');





    %% Laplacian

    [M,N,C] = size(imga);

    X = rgb2gray(imga);
    X = double(X);
    %% Laplacian
    Laplacian = zeros(M,N);
    for x=1:M
        for y=1:N
            if( (x>1 && x<M-1) &&(y>1 && y<N-1))
                gx2 = (X(x+1,y)-2*X(x,y)+X(x-1,y))/4;
                gy2 = (X(x,y+1)-2*X(x,y)+X(x,y-1))/4;
                Laplacian(x,y) = gx2+gy2;
            end
        end
    end
    %figure(3);
    %imshow(Laplacian>3);


    %% Tenenbaum Focus Measure
    Sobel = zeros(M,N);
    for x=1:M
        for y=1:N
            if(x>1 && x<M-1) && (y>1 && y<N-1)
                gx = (X(x+1,y)-X(x-1,y))/2;
                gy = (X(x,y+1)-X(x,y-1))/2;
                Sobel(x,y) = (gx^2 + gy^2)^2;
            end
        end
    end

    FMT = zeros(M,N);
    wn = 1;
    for x=1:M
        for y=1:N
            if((x>wn && x<M-wn) && (y>wn && y<N-wn))
                FMT(x,y) = sum(sum(Sobel(x-wn:x+wn, y-wn:y+wn)));
            end
        end
    end


    imga = imga.*uint8(FMT>2e4);
    la = single(rgb2gray(imga));
    [fa2, da2] = vl_sift(la, 'PeakThresh', 0, 'EdgeThresh', 10);

    %figure(1); 
    subplot(3,1,2)
    image(imga);
    hold on;
    x = fa2(1,:); y = fa2(2,:);
    plot(x,y, 'o');
    imga = imga.*uint8(Laplacian>3);
    la = single(rgb2gray(imga));
    [fa3, da3] = vl_sift(la, 'PeakThresh', 0, 'EdgeThresh', 10);
    
    %figure(2);
    subplot(3,1,3)
    image(imga);
    hold on;
    x = fa3(1,:); y = fa3(2,:);
    plot(x,y, 'o');
end