%%
% code for homography
% homography matrix H is the matrix that describes points on a FLAT PLANE,
% viewed in two different cameras. p'Hq = 0. 
% Author: Xiaocan Li
% date: Dec, 10, 2018
% www.github.com/hsiaotsan
%%

I_ad = imread('ad.jpg');
I_ad_copy = I_ad;
I_template = imread('template.jpg');
[r,c, ~] = size(I_template)
figure;
imshow(I_ad);

% (u2, v2) four corners of billboard
ad_corners_uv = [59 375 397 527;
                446 808 447 819;
                1 1 1 1
                ];
% (u1, v1) four corners of templates picture
template_corners_uv = [0 0 r r;
                        0 c 0 c;
                        1 1 1 1
                        ];

% Ah = b.  where h is flattened homography matrix, assuming h9=1.
b = [ad_corners_uv(1:2, 1);ad_corners_uv(1:2, 2);ad_corners_uv(1:2, 3);ad_corners_uv(1:2, 4)];

% A: coefficient matrix
A=[]
for i = 1:4
    aduv = ad_corners_uv(1:2, i);
    tmpuv = template_corners_uv(1:2, i)';
    Atmp = [tmpuv 1 0 0 0 -tmpuv*aduv(1);
        0 0 0 tmpuv 1 -tmpuv*aduv(2);
        ];
    A = [A;Atmp];
end

% solve 
h = A\b;
H9 = [h; 1];
H = reshape(H9, 3, 3)'; % transpose! Matlab reshapes vector to  matrix vertically 

% mapping template corners to ad picture, then normalize.
ad_corners_rec = H*template_corners_uv;
ad_corners_rec_norm = ad_corners_rec;
ad_corners_rec_norm(1, :) = ad_corners_rec_norm(1, :)./ad_corners_rec_norm(3, :);
ad_corners_rec_norm(2, :) = ad_corners_rec_norm(2, :)./ad_corners_rec_norm(3, :);
ad_corners_rec_norm

% set RGB value of template to ad picture at corresponding points
for u=1:r
    for v = 1:c
        uv3=H*[u;v;1];
        % don't forget normalize and round off to int.
        I_ad_copy(round(uv3(1)/uv3(3)), round(uv3(2)/uv3(3)), :) = I_template(u,v,:);
    end
end
imshow(I_ad_copy)