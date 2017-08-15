function [ frames ] = lynx_fk(joint_states)
%LIYNX_FK forward kinematics of lynx

    dh_parameters = [0, -pi/2, 3, 0; ...
                    5.75, 0, 0, -pi/2; 
                    7.375, 0, 0, pi/2; 
                    0, -pi/2, 0, -pi/2; 
                    0, 0, 4.125, 0];
    for i = 1:5
        dh_parameters(i, 4) = dh_parameters(i, 4) + joint_states(i);
    end
    
    frames = dh_to_homogenous(dh_parameters);
    
end

function [homogenous] = dh_to_homogenous(dh_parameters)
sz = size(dh_parameters, 1);
homogenous = zeros(sz, 4, 4);

for i=1:sz
    param = dh_parameters(i, :);
    a = param(1);
    al = param(2);
    d = param(3);
    th = param(4);
    x_trans = [1, 0, 0, a; ...
        0, cos(al), -sin(al), 0; ...
        0, sin(al), cos(al), 0; ...
        0, 0, 0,1];
    z_trans = [cos(th), -sin(th), 0, 0; ...
        sin(th), cos(th), 0, 0; ...
        0, 0, 1, d; ...
        0, 0, 0, 1];
    homo = z_trans * x_trans;
    homogenous(i, :, :) = homo;
    
end

end