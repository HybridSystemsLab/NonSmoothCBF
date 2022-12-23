function [b,gb] = Barrier(x, obst)
%%%%%%%%%%%%%%%%%%
    % Barrier
    % input: x , obst
        % x: position
        % obst: obstacle information
   
        
    % output: [b, gb]
        % b: barriers
        % gb: gradient
        
    % functions: compute Barriers and gradient of Barriers
%%%%%%%%%%%%%%%%%%

    theta = obst.theta;
    L = obst.L;
    W = obst.W;
    zo = obst.zo;
    Rs = obst.Rs;
    angles =[-cos(theta), -sin(theta);sin(theta), -cos(theta);cos(theta),sin(theta);-sin(theta), cos(theta)];
   
    m = 4;
    b = zeros(m,1);
    gb = angles;
    rec = [L+Rs;W+Rs;L+Rs;W+Rs] -angles * zo;
    b = angles * x +rec;
