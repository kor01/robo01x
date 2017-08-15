function [ linears, angulars ] = lynx_vk( theta, thetadot )
% velocity kinematics of lynx arm

    %% run forward kinematics
    dh_frames = lynx_fk(theta);
        
    T_x0 = zeros(5, 4, 4);
    
    current_T = eye(4);
    T_x0(1, :, :) = current_T;
    %% compute homogenous transforms w.r.t frame 0
    for i = 1:5
        current_T = current_T * squeeze(dh_frames(i, :, :));
        T_x0(i, :, :) = current_T;
    end
    
    %% compute angular velocity
    angulars = zeros(5, 3);
    
    current_angular = [0, 0, 0];
    current_axis = [0, 0, 1];
    for i = 1:5
        current_angular = current_angular + ...
            current_axis * thetadot(i);
        angulars(i, :) = current_angular;
        current_axis = T_x0(i, 1:3, 3);        
    end
        
    %% compute linear velocity of each origin
    
    linears = zeros(5, 3);
    current_velocity = zeros(1, 3);
    current_position = zeros(1, 3);
    for i = 1: 5
        position = T_x0(i, 1:3, 4) - current_position;
        current_velocity = current_velocity + ...
            cross(angulars(i, :), position);
        current_position = T_x0(i, 1:3, 4);
        linears(i, :) = current_velocity;
    end
end

