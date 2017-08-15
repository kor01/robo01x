function [ pos ] = lynx_pos( theta1, theta2, theta3, theta4, theta5, g )
%LYNX_FK The input to the function will be the joint
%    angles of the robot in radians, and the distance between the gripper pads in inches.
%    The output must contain 10 positions of various points along the robot arm as specified
%    in the question.

    %% YOUR CODE GOES HERE
    pos = zeros(10, 3);
    joint_states = [theta1, theta2, theta3, theta4, theta5];
    trans = lynx_fk(joint_states);
    
    T = eye(4);
    for i = 1:1:5
        %% position of origins from frame 0 to 4
        T(1:3, 4);
        pos(i, 1:end) = T(1:3, 4);
         T = T*squeeze(trans(i, 1:end, 1:end));
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
       pos(i + 5, 1:end) = position(1:3);   
    end
    
end

