clear all
close all

I = imread('img1_rect.tif');
imshow(I, [])

% these are points in model's coordinate system (inches)
P_M = [-2 -2 0 -2 -2 0;
    10 2 0 10 2 0;
    4 4 4 0 0 0;
    1 1 1 1 1 1];
% intrinsic params
f = 715;
cx = 354;
cy = 245;
K = [f 0 cx; 0 f cy; 0 0 1];

% y0: true location of points on image
y0 = [183; 147; 350; 133; 454; 144; 176; 258; 339; 275; 444; 286];

% make an initial guess of the pose
x = reshape([1.5 -1.0 0 0 0 30], [], 1)

% get predicted image points location
y = fProject(x, P_M, K);

% draw points, step 2 means one point has 2 values. 16 means rectangle size
for i = 1:2:length(y)
    rectangle('Position', [y(i)-8 y(i+1)-8 16 16], 'FaceColor', 'r');
end

for i =1:500
    fprintf('\nIteration %d\nCurrent pose:\n', i);
    disp(x);
    
    % get predicted image points
    y = fProject(x, P_M, K);
    imshow(I, [])
    
    for i = 1:2:length(y)
    rectangle('Position', [y(i)-8 y(i+1)-8 16 16], 'FaceColor', 'r');
    end
%     pause(0.4);
    
    % estimate Jacobian
    e = 0.00001;
    J(:,1) = ( fProject(x+[e;0;0;0;0;0],P_M,K) - y )/e;
    J(:,2) = ( fProject(x+[0;e;0;0;0;0],P_M,K) - y )/e;
    J(:,3) = ( fProject(x+[0;0;e;0;0;0],P_M,K) - y )/e;
    J(:,4) = ( fProject(x+[0;0;0;e;0;0],P_M,K) - y )/e;
    J(:,5) = ( fProject(x+[0;0;0;0;e;0],P_M,K) - y )/e;
    J(:,6) = ( fProject(x+[0;0;0;0;0;e],P_M,K) - y )/e;
    dy = y - y0;
    fprintf('Residual error: %f\n', norm(dy));
    
    dx = pinv(J)*dy;
    if norm(J'*dy) < 1e-6
        break;
    end
    
    % update x, gradient descent. note that dy = y -y0, not y
    x = x - dx;
end

u0 = fProject(x, [0;0;0;1], K); % project origin to image
uX = fProject(x, [1;0;0;1], K); % project x-axis vector to image
uY = fProject(x, [0;1;0;1], K);
uZ = fProject(x, [0;0;1;1], K);

line([u0(1) uX(1)], [u0(2) uX(2)], 'Color', 'r', 'LineWidth', 3);
line([u0(1) uY(1)], [u0(2) uY(2)], 'Color', 'g', 'LineWidth', 3);
line([u0(1) uZ(1)], [u0(2) uZ(2)], 'Color', 'b', 'LineWidth', 3);