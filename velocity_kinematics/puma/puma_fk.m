function [ frames ] = puma_fk( joint_states )
%PUMA_FK forward kinematics for puma

    a = 13.0; b = 2.5; c = 8.0; d = 2.5; e = 8.0; f = 2.5;
    dh_parameters = [0, pi/2, a, 0; ...
                    c, 0, -b, 0; ...
                    0, -pi/2, -d, 0; ...
                    0, pi/2, e, 0; ...
                    0, -pi/2, 0, 0
                    0, 0, f, 0];
                
    dh_parameters(:, 4) = dh_parameters(:, 4) + joint_states';
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