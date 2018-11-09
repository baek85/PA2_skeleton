function [matches, scores] = feature_extraction(la, lb)

%Ia = vl_impattern('roofs1');
%lb = vl_impattern('roofs2');

la = single(rgb2gray(la));
lb = single(rgb2gray(lb));
[fa, da] = vl_sift(la);
[fb, db] = vl_sift(lb);

perm = randperm(size(f,2));
sel = perm(1:50);
h1 = vl_plotframe(fa(:,sel));
h2 = vl_plotframe(fa(:,sel));
set(h1, 'color', 'k', 'linewidth', 3);
set(h2, 'color', 'y', 'linewidth', 2);

h3 = vl_plotsiftdescriptor(da(:,sel), fa(:,sel));
set(h3, 'color', 'g');


[matches, scores] = vl_ubcmatch(da, db);
end