%% 
% x = [ax; ay; az; tx; ty; tz] model-to-camera pose param
% P_M = [X; Y; Z; 1] points in model coordinate
% K intrinsic matrix
% p predicted points on image
%% project 3D points onto image
function p = fProject(x, P_M, K)
% get pose params
ax = x(1); ay = x(2); az = x(3); 
tx = x(4); ty = x(5); tz = x(6);

% rotation matrix, model to camera
Rx = [1 0 0;0 cos(ax) -sin(ax); 0 sin(ax) cos(ax)];
Ry = [cos(ay) 0 sin(ay); 0 1 0; -sin(ay) 0 cos(ay)];
Rz = [cos(az) -sin(az) 0; sin(az) cos(az) 0; 0 0 1];
R = Rz*Ry*Rx;

% extrinsic camera matrix
Mext = [R [tx;ty;tz]];

% project point
ph = K*Mext*P_M;
ph(1, :) = ph(1, :)./(ph(3, :));
ph(2, :) = ph(2, :)./(ph(3, :));
ph = ph(1:2, :);
p = reshape(ph, [], 1);
return


