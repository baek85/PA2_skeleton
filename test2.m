load all.mat

num_inliers = 0;
num_Match(ref, pair)
for a=1:number_of_iterations_for_5_point
    perm = randperm(num_Match(ref, pair));
    ref_x = perm(1:5);
    pair_x = Match(ref, ref_x, pair);
    Q1 = inv(K)*[Feature(1:2,ref_x,ref); ones(1,5)];
    Q2 = inv(K)*[Feature(1:2,pair_x,pair); ones(1,5)];

    Evec   = calibrated_fivepoint( Q1,Q2);
    for i=1:size(Evec,2)
      E = reshape(Evec(:,i),3,3);
      % Check determinant constraint! 
      %det( E)
      % Check trace constraint
      %2 *E*transpose(E)*E -trace( E*transpose(E))*E
      % Check reprojection errors
      %diag( Q1'*E*Q2)
    end
    E; % find out
    F = inv(K')*E*inv(K);

    ref_x = 1:num_Match(ref, pair);
    pair_x =  Match(ref, ref_x, pair);
    X1 = [Feature(1:2, ref_x, ref); ones(1, num_Match(ref, pair))];
    X2 = [Feature(1:2, pair_x, pair); ones(1, num_Match(ref, pair))];


    d1 = sum((X1-F*X2).*(X1-F*X2), 1);
    d2 = sum((X2-F*X1).*(X2-F*X1), 1);
    d = d1+d2;
    if(sum(d<threshold_of_distance)> num_inliers)
        E_ret = E;
        num_inliers = sum(d<threshold_of_distance)
    end
end

E