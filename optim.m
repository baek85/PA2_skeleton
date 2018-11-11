%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This is the skeleton code of PA2 in EE735 Computer Vision.              %
% It will help you to implement the Structure-from-Motion method easily.  %
% Using this skeleton is recommended, but it's not necessary.             %
% You can freely modify it or you can implement your own program.         %
% If you have a question, please send me an email to shim@rcv.kaist.ac.kr%
%                                                        TA. Sunghoon Im  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all;
clear all;

addpath('Givenfunctions');
addpath(genpath('vlfeat-0.9.21'))
data_dir='Data';
%data_dir = 'Data2';
%% Define constants and parameters
% Constants ( need to be set )
number_of_pictures                  = 32;     % number of input pictures
%number_of_pictures = 39;
number_of_iterations_for_5_point    = 1000;
number_of_iterations_for_3_point    = 0;
number_of_iterations_for_LM         = 0;

% Thresholds ( need to be set )
threshold_of_distance = 1; 

% Assumptions ( need to be set )
max_number_of_features  = 1500; % maximum number of features in 1 image
max_number_of_points    = 50000; % maximum number of reconstructed points

% Matrices
K               = [ 1698.873755 0.000000     971.7497705;
                    0.000000    1698.8796645 647.7488275;
                    0.000000    0.000000     1.000000 ];
% K               = [ 14694.18649 0.000000  2669.15048;
%                     0.000000    8083.86985   2084.23391;
%                     0.000000    0.000000     1.000000 ];
                % = intrinsic parameter matrix. 3 x 3
num_Feature     = zeros(1, number_of_pictures);                         
                % = number of features in each images. 1 x (# picture)
Feature         = zeros(4, max_number_of_features, number_of_pictures);     
                % = SIFT features in each images. 4 x (# feature) x (# picture) containing [ x y s th ]
Descriptor      = zeros(128, max_number_of_features, number_of_pictures);
                % = SIFT descriptors in each images. 128 x (# feature) x (# picture)
num_Match       = zeros(number_of_pictures, number_of_pictures);          
                % = number of matches. (# picture) x (# picture)
Match           = zeros(number_of_pictures, max_number_of_features, number_of_pictures);
                % = corresponding feature index in second picture for a feature of first picture. (# picture) x (# feature) x (# picture)
X               = zeros(6, max_number_of_points);                                 
                % = reconstructed 3D points. 6 x (# point) containing [ x y z r g b ] (float)
X_exist         = zeros(1, max_number_of_points);                           
                % = 1 : X is exist. 0 : not. 1 x (# point)
Feature2X       = zeros(number_of_pictures, max_number_of_features);
                % = feature to 3D pointindex. (# picture) x (# feature)
X2Feature       = zeros(max_number_of_points, number_of_pictures);        
                % = 3D point to feature index. (# point) x (# picture)
Image_selected  = zeros(1, number_of_pictures);                      
                % = 1 : image is selected, 0 : not. (# picture)
R               = zeros(3, 3, number_of_pictures);                                
                % = Camera rotation R of images. 3 x 3 x (# picture)
T               = zeros(3, number_of_pictures);                      
                % = Camera translation t of images. 3 x (# picture)
eps = 0;
% ADVICE : These matrices seem very difficult, but you will need sum data structures like them.

list = dir(data_dir);

%% Feature extraction and matching
% Load images and extract features and find correspondences.
% Fill num_Feature, Feature, Descriptor, num_Match and Match
% hints : use vl_sift to extract features and get the descriptors.
%        use vl_ubcmatch to find corresponding matches between two feature sets.
% 
% for idx=1:number_of_pictures
%     list = dir(data_dir);
%     path = fullfile(data_dir, list(idx+2).name);
%     tmp = imread(path); tmp = single(rgb2gray(tmp));
%     [f, d] = vl_sift(tmp,'PeakThresh', 0.3, 'EdgeThresh', 10);
%     num_Feature(idx) = size(f,2);
%     perm = randperm(size(f,2));
%     sel = perm(1:max_number_of_features);
%     Feature(:,1:max_number_of_features, idx) = f(:, sel);
%     Descriptor(:,1:max_number_of_features, idx) = d(:,sel);
% 
% end
% len = zeros(number_of_pictures);
% 
% for a=1:number_of_pictures-1
%     for b=a+1:number_of_pictures
%         [matches, scores] = vl_ubcmatch(Descriptor(:,:,a), Descriptor(:,:,b));
%         num_Match(a,b) = size(matches,2);
%         num_Match(b,a) = size(matches,2);
%         Match(a,1:size(matches,2),b) = matches(1,:);
%         Match(b,1:size(matches,2),a) = matches(2,:);
%         
%     end
% end
%save 0.3.mat
load 0.3.mat

X4               = zeros(7, max_number_of_points); 
number_of_pictures                  = 32;     % number of input pictures
number_of_iterations_for_5_point    = 1000;
number_of_iterations_for_3_point    = 1000;
number_of_iterations_for_LM         = 0;

% Thresholds ( need to be set )

threshold_of_distance = 1; 
%% Initialization step (for 2 views)
% Find the best reference image. 
% It is reasonable to use the image which has the largest sum of # correspondences to other images.
% Let's call it 'ref'
[M, ref] = max(sum(num_Match, 1));
ref; % find out

% Find the best pair of reference image.
% It will be the one that has the largest # correspondences to the reference image.
% Let's call it 'pair'
[M, pair] = max(num_Match(ref,:));
pair; % find out

%ref = 1; pair = 2;
Image_selected(ref)=1;
Image_selected(pair)=1;

% Give ref's R|T = I|0
R(:, :, ref) = eye(3);
T(:, ref) = zeros(3, 1);

% Estimate E using 8,7-point algorithm or calibrated 5-point algorithm and RANSAC
num_inliers = 0;
for a=1:number_of_iterations_for_5_point
    perm = randperm(num_Match(ref, pair));
    perm = perm(1:5);
    ref_x = Match(ref,perm, pair);
    pair_x = Match(pair, perm, ref);
    
   %%  Q1 = right, Q2 = left
    Q1 = K \ [Feature(1:2,pair_x,pair); ones(1,5)];
    Q2 = K \ [Feature(1:2,ref_x,ref); ones(1,5)];    

    Evec   = calibrated_fivepoint(Q1,Q2);
    for i=1:size(Evec,2)
        Etmp = reshape(Evec(:,i),3,3);
        % Check determinant constraint! 
        term1 = det(Etmp);
        % Check trace constraintk
        term2 =  (2 * (Etmp * Etmp') - trace(Etmp * Etmp') * eye(3)) * Etmp;
        % Check reprojection errors
        term3 = diag(Q1' * Etmp * Q2);
        if(abs(term1) < 1e-10 && abs(sum(sum(term2))) < 1e-10 && abs(sum(term3)) < 1e-10)
            % find out
            F = K' \ Etmp / K;
            perm = 1:(num_Match(ref, pair));
            ref_x = Match(ref,perm, pair);
            pair_x = Match(pair,perm, ref);

            tmp = zeros(1,num_Match(ref,pair));
            for ii = 1:num_Match(ref,pair)
                X1 = [Feature(1:2,pair_x(ii), pair);1];
                X2 = [Feature(1:2,ref_x(ii),ref);1];

                term1 = F * X2;
                term2 = F' * X1;
                d2 = (X1' * F * X2)^2 * (1 / (term1(1)^2 + term1(2)^2) + 1 / (term2(1)^2 + term2(2)^2));
                tmp(ii) = d2;
            end
            sum(tmp < threshold_of_distance);
            if (sum(tmp < threshold_of_distance) > num_inliers)
                Eret = Etmp;
                num_inliers = sum(tmp<threshold_of_distance);
                fprintf('5-point algorithm iters : %d    ', a);
                fprintf('number of inliers = %d  / %d\n', num_inliers, num_Match(ref, pair));
            end
        end
        
    end
end
E = Eret;

%load E.mat

% Decompose E into [Rp, Tp]

%[U, S, V] = svd(sqrt(2)*E);
[U, S, V] = svd(E);
%U = U * S(1,1);
%V = V*S(1,1);
W = [0 -1 0; 1 0 0; 0 0 1];  u3 = U * [0,0,1]';

%% Find optimal camera pose
P = zeros(4,3,4);
P(1,:,:) = [U * W * V', u3];
P(2,:,:) = [U * W * V', -u3];
P(3,:,:) = [U * W' * V', u3];
P(4,:,:) = [U * W' * V', -u3];

num_inliers = zeros(4,1);
X3tmp = zeros(4, 4, num_Match(ref,pair));

%% Ppair1 , Ppair4  -> (depth >0)
for pidx = 1:4
    Ptmp = squeeze(P(pidx,:,:));
    Rp = Ptmp(:,1:3); % find out
    Tp = Ptmp(:,4); % find out

    % (Optional) Optimize Rp and Tp

    % Give pair's R|T = Rp|Tp
    R(:, :, pair) = Rp;
    T(:, pair) = Tp;

    % Reconstruct 3D points using triangulation
    P1 = K * [R(:,:,ref), T(:,ref)];
    p1T1 = P1(1,:);
    p2T1 = P1(2,:);
    p3T1 = P1(3,:);
    P2 = K * [R(:,:,pair), T(:,pair)];
    p1T2 = P2(1,:);
    p2T2 = P2(2,:);
    p3T2 = P2(3,:);

    for i = 1:num_Match(ref,pair)
        X1 = [Feature(1:2,Match(ref , i, pair),ref);1];
        X2 = [Feature(1:2,Match(pair, i, ref), pair);1];
        A = [ ...
            X1(1) * p3T1 - p1T1; ...
            X1(2) * p3T1 - p2T1; ...
            X2(1) * p3T2 - p1T2; ...
            X2(2) * p3T2 - p2T2 ...
        ];
        [U, O, V] = svd(A);
        Xtmp = V(:,end);
        Xtmp = Xtmp/(Xtmp(4)+eps);
        
        Xcam1 = [[R(:,:,ref),T(:,ref)];0 0 0 1]*Xtmp;
        Xcam2 = [[R(:,:,pair),T(:,pair)];0 0 0 1]*Xtmp;

        x1 = [R(:,:,ref),T(:,ref)]*Xtmp;
        x2 = [R(:,:,pair),T(:,pair)]*Xtmp;
        depth1 = sign(det(R(:,:,ref)))*x1(3)/Xtmp(4);
        depth2 = sign(det(R(:,:,pair)))*x2(3)/Xtmp(4);
        %if(depth1 >0 && depth2 >0)
        if(Xcam1(3)>0 && Xcam2(3) >0)
            num_inliers(pidx) = num_inliers(pidx) +1;
        end

        x1 = P1*Xtmp;
        x2 = P2*Xtmp;
        x1 = x1/(x1(3)+eps);
        x2 = x2/(x2(3)+eps);

        X3tmp(pidx,:,i) = Xtmp;
        
%         if(num_inliers(pidx)>0.1*num_Match(ref,pair))
%             break;
%         end   
    end  
end
num_inliers
[a, pidx]= max(num_inliers);

P2 = squeeze(P(pidx,:,:));
R(:, :, pair) = P2(:,1:3);
T(:, pair) = P2(:,4);

P1 = K * [R(:,:,ref),T(:,ref)];
P2 = K*  [R(:,:,pair),T(:,pair)];
perm = 1:num_Match(ref, pair);
X1 = [Feature(1:2,Match(ref , perm, pair),ref);ones(1,num_Match(ref,pair))];
X2 = [Feature(1:2,Match(pair , perm, ref),pair);ones(1,num_Match(ref,pair))];
x3 = squeeze(X3tmp(pidx,:,perm));
x1 = P1*x3;
x1 = x1./(x1(3,:)+eps);
x2 = P2*x3;
x2 = x2./(x2(3,:)+eps);
d = (x1(1,:)-X1(1,:)).^2 + (x1(2,:)-X1(2,:)).^2 + (x2(1,:)-X2(1,:)).^2 + (x2(2,:)-X2(2,:)).^2;
num_inliers = sum(d<threshold_of_distance);

num_inliers

diff = zeros(1, num_Match(ref,pair));
for i = 1:num_Match(ref,pair)
    X1 = [Feature(1:2,Match(ref , i, pair),ref);1];
    X2 = [Feature(1:2,Match(pair, i, ref), pair);1];
    x3 = X3tmp(pidx,:,i)';
    D = sum(((P1*x3)./(P1(3,:)*x3+eps) - X1).^2 + ((P2*x3)./(P2(3,:)*x3+eps) - X2).^2, 1);

    disp("D = "+ D);
    fun = @(x)[sqrt(sum(((P1*[x(1); x(2); x(3);1])./(P1(3,:)*[x(1); x(2); x(3);1]+eps) - X1).^2));...
            sqrt(sum(((P2*[x(1); x(2); x(3);1])./(P2(3,:)*[x(1); x(2); x(3);1]+eps) - X2).^2))];
    x0 = x3(1:3);
    options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective');
    %options.Algorithm = 'levenberg-marquardt';
    [x, resnorm] = lsqnonlin(fun,x0,[],[],options)
    x = [x;1];
    d = sum(((P1*x)./(P1(3,:)*x+eps) - X1).^2 + ((P2*x)./(P2(3,:)*x+eps) - X2).^2, 1);
    disp("d = "+ d);
    X3tmp(pidx,:,i) = x';
    disp("diff = " +(D-d));
    diff(i) = D-d;

end
sum(diff)
RTret = [R(:,:,pair),T(:,pair)];

P1 = K*[R(:,:,ref),T(:,ref)];
P2 = K*RTret(1:3,:);
Cam1 = zeros(3,num_Match(ref,pair));
Cam2 = zeros(3,num_Match(ref,pair));
path = fullfile(data_dir, list(ref+2).name);
imga = imread(path);
path = fullfile(data_dir, list(pair+2).name);
imgb = imread(path);
[h, w, c] = size(imga);
idx3d=1;

% figure(1); 
% subplot(2,1,1)
% image(imga);
% hold on;
% x = X1(1,:); y = X1(2,:);
% 
% plot(x,y, 'o');
% subplot(2,1,2)
% imshow(imgb);
% x = X2(1,:); y = X2(2,:);
% hold on;
% plot(x,y, 'o');
% 
% 
% fa = round(x1); fb = round(x2);
% figure(2); 
% subplot(2,1,1)
% image(imga);
% hold on;
% x = fa(1,:); y = fa(2,:);
% 
% plot(x,y, 'o');
% subplot(2,1,2)
% imshow(imgb);
% x = fb(1,:); y = fb(2,:);
% hold on;
% plot(x,y, 'o');


for i = 1:num_Match(ref,pair)
    X1 = [Feature(1:2,Match(ref , i, pair),ref);1];
    X2 = [Feature(1:2,Match(pair, i, ref), pair);1];
    %Xtmp = [squeeze(X3tmp(pidx,:,i))'; 1];
    Xtmp = squeeze(X3tmp(pidx,:,i))';
    x1 = P1*Xtmp;  x1= x1/(x1(3)+eps);
    x2 = P2*Xtmp; x2 = x2/(x2(3)+eps);
    d = (x1(1)-X1(1)).^2 + (x1(2)-X1(2)).^2 + (x2(1)-X2(1)).^2 + (x2(2)-X2(2)).^2;
    if(d<threshold_of_distance)
        x1 = round(x1);
        x = max(min(xi(2), w), 1);
        y = max(min(xi(1), h), 1);

        X(1:3,idx3d)= Xtmp(1:3);
        X(4:6,idx3d) = double(imga(x, y,:))/255;
        X_exist(idx3d) = 1;
        
        Feature2X(ref, Match(ref,i,pair)) = idx3d;
        Feature2X(pair, Match(pair, i, ref)) = idx3d;
        
        X2Feature(idx3d, ref) = Match(ref, i, pair);
        X2Feature(idx3d, pair) = Match(pair, i, ref);
        
        Xcam1 = [[R(:,:,ref),T(:,ref)];0 0 0 1]*Xtmp;
        Xcam2 = [[R(:,:,pair),T(:,pair)];0 0 0 1]*Xtmp;
        Cam1(:,i) = Xtmp(1:3) - Xcam1(1:3);
        Cam2(:,i) = Xtmp(1:3) - Xcam2(1:3);
        idx3d = idx3d+1;
    end
end

old2d = Match(ref, 1:num_Match(pair,ref), pair);
Xtmp = Feature2X(ref,old2d);
old2d = old2d.*(Xtmp>0); old2d = old2d(find(old2d));
new2d = Match(pair, 1:num_Match(pair,ref), ref).* (Xtmp>0);
new2d = new2d(find(new2d));

Feature2X(ref, old2d) - Feature2X(pair, new2d);


% Estimate pose R|T using 6-point DLT or 3-point algorithm.
idx2 = new2d;
idx3 = Feature2X(ref, old2d);

num_inliers = 0;
for i=1:1000
    perm = randperm(length(idx2));
    perm = perm(1:3);
    nx = K \ [Feature(1:2,idx2(perm),pair); ones(1,3)];
    x3 = X(1:3,idx3(perm))';
    data = [nx',x3];
    RT=PerspectiveThreePoint(data);
    if(RT ~= -1)
        P1 = K*[R(:,:, ref),T(:,ref)];
        for k = 1:size(RT,1)/4
            P3 = K*RT(1+4*(k-1):3+4*(k-1),:);
            X1 = [Feature(1:2,old2d, ref); ones(1,length(idx2))];
            X2 = [Feature(1:2,idx2, pair); ones(1,length(idx2))];
            x3 = [X(1:3, idx3);ones(1,length(idx2))];   % P3 에 대해서는 잘 변환함  P2에 대해서는 이상하게 변환
            x1 = P1*x3;
            x1 = x1./(x1(3,:)+eps);
            d1 = (x1(1,:)-X1(1,:)).^2 + (x1(2,:)-X1(2,:)).^2;
            x2 = P3*x3;
            x2 = x2./(x2(3,:)+eps);
            d2 = (x2(1,:)-X2(1,:)).^2 + (x2(2,:)-X2(2,:)).^2;
            if(sum(d1+d2<threshold_of_distance) > num_inliers)
                RTret = RT(1+4*(k-1):3+4*(k-1),:);
                num_inliers = sum(d1+d2<threshold_of_distance);
                fprintf('3-point algorithm iters = %d    number of inliers = %d / %d\n', i, num_inliers, length(idx2));
            end
        end
    end
end

P3 = K*RTret;
p1T1 = P1(1,:);
p2T1 = P1(2,:);
p3T1 = P1(3,:);
p1T2 = P3(1,:);
p2T2 = P3(2,:);
p3T2 = P3(3,:);
        
Cam1 = mean(Cam1,2);
Cam2 = mean(Cam2,2);
X(:,idx3d) = [[0.000, 0.000, 0.000]';[1, 0, 0]']; X_exist(idx3d)=1;
idx3d = idx3d+1;
X(:,idx3d) = [Cam1-Cam2;[1, 0, 0]']; X_exist(idx3d)=1;
idx3d = idx3d+1;

%X(:,idx3d+3:end) = zeros(size(X(:,idx3d+3:end)));
X; % find out
X_exist; % find out


% Save 3D points to PLY
filename = sprintf('02_%02d_%02dviews.ply', ref, pair);
SavePLY(filename, X);