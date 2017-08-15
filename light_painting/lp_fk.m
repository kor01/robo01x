function [ frames ] = lp_fk( theta1, theta2, theta3, theta4, theta5, theta6 )
%LP_FK Summary of this function goes here
%   Detailed explanation goes here
    frames = puma_fk([theta1, theta2, theta3, theta4, theta5, theta6]);
    eef = eye(4);
    sz = size(frames, 1);
    
    frames = zeros(sz, 4, 4);
    for i=1:sz
        eef = eef * squeeze(frames(i, :, :));
        frames(i, :, :) = eef;
    end

end

