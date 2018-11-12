old2 = zeros(number_of_pictures, max_number_of_features);
    new2 = zeros(number_of_pictures, max_number_of_features);
    idx3 =[];
    
    for idx = 1:length(I)
        old2d = Match(old(idx), 1:num_Match(old(idx),new), new);
        Xtmp = Feature2X(old(idx),old2d);
        old2d = old2d.*(Xtmp>0); old2d = old2d(find(old2d));
        new2d = Match(new, 1:num_Match(old(idx),new),old(idx)).* (Xtmp>0);
        new2d = new2d(find(new2d));
        Feature2X(new, new2d) = Feature2X(old(idx),old2d);
        old2(old(idx), 1:length(old2d)) = old2d;
        new2(old(idx), 1:length(new2d)) = new2d;
        %idx2(old(idx), 1:length(new2d)) = new2d;
        idx3 = [idx3, Feature2X(new,new2d)];
    end
        
        % Estimate pose R|T using 6-point DLT or 3-point algorithm.
        
        %idx2 = new2d;
        %idx3 = Feature2X(new,new2d);
        
        num_inliers = 0;
        for i=1:200000
            perm = randperm(length(idx3));
            perm = perm(1:3);
            %old_x = old2d(perm);
            %new_x =  new2d(perm);
            nx = [];
            for iter = 1:3
                matched = find(X2Feature(idx3(perm(iter)),:));
                sel = randi(length(matched));
                nx = [nx, K \ [Feature(1:2, X2Feature(idx3(perm(iter)), matched(sel)), new); 1]];
            end
            %nx = K \ [Feature(1:2,new_x,new); ones(1,3)];
            x3 = X(1:3, idx3(perm))';
            data = [nx',x3];
            RT=PerspectiveThreePoint(data);
            if(RT ~= -1)
                
                %%RT size ==4 ÀÏ¶§
                for k = 1:size(RT,1)/4
                    P2 = K*RT(1+4*(k-1):3+4*(k-1),:);
                    d = [];
                    for idx = 1:length(I)
                        P1 = K*[R(:,:,old(idx)),T(:,old(idx))];
                        len_old2d = sum(old2(old(idx),:)>0);
                        old2d = old2(old(idx),1:len_old2d);
                        new2d = new2(old(idx),1:len_old2d);
                        X1 = [Feature(1:2,old2d, old(idx)); ones(1,len_old2d)];
                        X2 = [Feature(1:2,new2d,new); ones(1,len_old2d)];
                        %Feature2X(old(idx), old2d) - Feature2X(new, new2d);
                        new3d = Feature2X(new, new2d);
                        x3 = [X(1:3, new3d);ones(1,len_old2d)];
                        x1 = P1*x3;
                        x1 = x1./(x1(3,:)+eps);
                        d1 = (x1(1,:)-X1(1,:)).^2 + (x1(2,:)-X1(2,:)).^2;
                        x2 = P2*x3;
                        x2 = x2./(x2(3,:)+eps);
                        d2 = (x2(1,:)-X2(1,:)).^2 + (x2(2,:)-X2(2,:)).^2;
                        d = [d, d1+d2];
                    end
                    if(sum(d<threshold_of_distance) > num_inliers)
                            RTret = RT(1+4*(k-1):3+4*(k-1),:);
                            num_inliers = sum(d<threshold_of_distance);
                            fprintf('3-point algorithm iters = %d    number of inliers = %d / %d\n', i, num_inliers, length(idx2));
                        end
                end
            end
        end