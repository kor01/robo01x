function [ pos ] = puma_pos( theta1, theta2, theta3, theta4, theta5, theta6, g  )
%PUMA_POS Summary of this function goes here
%   Detailed explanation goes here

    pos = zeros(12, 3);
    joint_states = [theta1, theta2, theta3, theta4, theta5, theta6];
    trans = puma_fk(joint_states);
    
    extended_trans = zeros(7, 4, 4);
    
    extended_trans(1, :, :) = trans(1, :, :);
    extended_trans(2, :, :) = [1, 0, 0, 0; 
                               0, 1, 0, 0; 
                               0, 0, 1, -2.5; 
                               0, 0, 0, 1];
    extended_trans(3:end, :, :) = trans(2:end, :, :);
    extended_trans(3, 3, 4) = 0;
        
    T = eye(4);
    for i = 1:1:7
        %% position of origins from frame 0 to 4
        pos(i, 1:end) = T(1:3, 4);
         T = T*squeeze(extended_trans(i, 1:end, 1:end));
    end
    
    
    e = 1.125;
    points = [0 0 -e 1;... 
            g/2 0 -e 1;...
            -g/2 0 -e 1; ...
            g/2 0 0 1; ...
            -g/2 0 0 1];
    
    for i = 1:1:5
       %% absolute position of points
       point = squeeze(points(i, 1:end))';
       position = T*point;
       pos(i + 7, 1:end) = position(1:3);   
    end
    
end

