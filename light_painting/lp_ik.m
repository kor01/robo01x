function [ ik_sol ] = lp_ik(  x, y, z, R )
%LP_IK inverse kinematics for puma

f = 2.5;

zaxis = R(:, 3);
pos = [x, y, z];

wrist_center = pos - zaxis * f;
wcx = wrist_center(1);
wcy = wrist_center(2);
wcz = wrist_center(3);

% case that theta1 in [-pi/2, pi/2]
predicate = -25 * wcx^2 + wcx^2*wcy^2 + wcx^4;
assert(predicate >=0, 'unfeasible for theta1')


%% cosine and sine of theta1
s11 = (-5*wcx - sqrt(predicate)) / (wcx^2 + wcy^2);
s12 = (-5*wcx + sqrt(predicate)) / (wcx^2 + wcy^2);

c11 = (wcx*s11 + 0.5) / wcy;
c12 = (wcx*s12 + 0.5) / wcy;

%% for each solution of theta1, compute other joint angles
    
ik_sol = 0;    

end


function [theta2, theta3] = solve_shoulder_elbow( theta1, wc)

%% inversely apply rotation1 to wc
    rotation = rotz(theta1);
    rot_wc = rotation'*wc;
    wcx = rot_wc(1);
    wcz = rot_wc(3);
    
    

end