function [ linears, angulars ] = puma_vk( theta, thetadot )
%PUMA_IK puma velocity kinematics

    %% run forward kinematics
    dh_frames = puma_fk(theta);
        
    T_x0 = zeros(6, 4, 4);
    
    current_T = eye(4);
    T_x0(1, :, :) = current_T;
    %% compute homogenous transforms w.r.t frame 0
    for i = 1:6
        current_T = current_T * squeeze(dh_frames(i, :, :));
        T_x0(i, :, :) = current_T;
    end
    
    %% compute angular velocity
    angulars = zeros(6, 3);
    
    current_angular = [0, 0, 0];
    current_axis = [0, 0, 1];
    for i = 1:6
        current_angular = current_angular + ...
            current_axis * thetadot(i);
        angulars(i, :) = current_angular;
        current_axis = T_x0(i, 1:3, 3);        
    end
        
    %% compute linear velocity of each origin
    
    linears = zeros(6, 3);
    current_velocity = zeros(1, 3);
    current_position = zeros(1, 3);
    for i = 1: 6
        position = T_x0(i, 1:3, 4) - current_position;
        current_velocity = current_velocity + ...
            cross(angulars(i, :), position);
        current_position = T_x0(i, 1:3, 4);
        linears(i, :) = current_velocity;
    end
end
