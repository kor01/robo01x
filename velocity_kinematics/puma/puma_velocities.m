function [ v06, w06 ] = puma_velocities( thetas, thetadot )
%PUMA_VELOCITIES The input to the function will be:
%    thetas: The joint angles of the robot in radians - 1x6 matrix
%    thetadot: The rate of change of joint angles of the robot in radians/sec - 1x6 matrix
%    The output has 2 parts:
%    v06 - The linear velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    w06 - The angular velocity of frame 6 with respect to frame 0, expressed in frame 0.
%    They are both 1x3 matrices of the form [x y z] for a vector xi + yj + zk

    %% YOUR CODE GOES HERE
    [linears, angulars] = puma_vk(thetas, thetadot);
    v06 = linears(end, :);
    w06 = angulars(end, :);
end