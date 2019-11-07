clear all;
close all;
clc;
%%
posX = 0.75;
posY = 0.0;
posZ = 0.2;
rotX = 0.0;
rotY = 0.0;



%%
robot = robotics.RigidBodyTree('DataFormat','row','MaxNumBodies',5);

L1= 0.2;
L2 = 0.3;
L3 = 0.3;
L4 = 0.1;
Lg = 0.05;
L5 = Lg;
%Base
body = robotics.RigidBody('link1');
joint = robotics.Joint('joint1', 'revolute');
setFixedTransform(joint,trvec2tform([0 0 0]));
joint.PositionLimits = [-pi/2 pi/2];
joint.JointAxis = [0 0 1];
body.Joint = joint;
addBody(robot, body, 'base');
%link 1
body = robotics.RigidBody('link2');
joint = robotics.Joint('joint2','revolute');
setFixedTransform(joint, trvec2tform([0,0,-L1]));
joint.PositionLimits = [-pi/8 pi];
joint.JointAxis = [1 0 0];

body.Joint = joint;
addBody(robot, body, 'link1');

body = robotics.RigidBody('link3');
joint = robotics.Joint('joint3','revolute');
setFixedTransform(joint, trvec2tform([0,L2,0]));
joint.JointAxis = [1 0 0];
joint.PositionLimits = [-pi/2 pi/2];
body.Joint = joint;
addBody(robot, body, 'link2');

body = robotics.RigidBody('link4');
joint = robotics.Joint('joint4','revolute');
setFixedTransform(joint, trvec2tform([0,L3,0]));
joint.JointAxis = [1 0 0];
joint.PositionLimits = [-pi/2 pi/2];
body.Joint = joint;
addBody(robot, body, 'link3');

body = robotics.RigidBody('link5');
joint = robotics.Joint('joint5','revolute');
setFixedTransform(joint, trvec2tform([0,L4,0]));
joint.JointAxis = [0 1 0];
joint.PositionLimits = [-pi/2 pi/2];
body.Joint = joint;
addBody(robot, body, 'link4');

body = robotics.RigidBody('tool');
joint = robotics.Joint('fix1','fixed');
setFixedTransform(joint, trvec2tform([0, Lg, 0]));
body.Joint = joint;
addBody(robot, body, 'link5');


showdetails(robot)
%getTransform(robot,config,'tool','base')
show(robot);

%%
% ik = robotics.InverseKinematics('RigidBodyTree', robot);
% qInitial = homeConfiguration(robot);
% weights = [0, 0, 0, 1, 1, 1];
% endEffector = 'tool';
% point = [0.1 0.6 -0.2];
% %%
% qSol = ik(endEffector,trvec2tform(point),weights,qInitial);
% qInitial = qSol;
% qs(:) = qSol;
% show(robot);
%%




eul = [0 rotY rotX];
rot = eul2rotm(eul);
start = [rot(1,1) rot(1,2) rot(1,3) posX
            rot(2,1) rot(2,2) rot(2,3) posY
            rot(3,1) rot(3,2) rot(3,3) posZ
             0 0 0 1];

%start = T;
q0 = homeConfiguration(robot);
%ndof = length(q0);
%qs = zeros(count, ndof);



ik = robotics.InverseKinematics('RigidBodyTree', robot);
weights = [0.1, 0.1, 0, 0.1, 0.1, 0.1];
endEffector = 'tool';

qInitial = q0; % Use home configuration as the initial guess

qSol = ik(endEffector,start,weights,qInitial)


%%

x = 0.2;
y = 0.2;
z = -L1;
p = 0;
r = 0;
s = [0 0 0 0 0];
while 1
    s(5) = r;
    s(1) = atan2(y, x);
    L = sqrt(x^2 + y^2);
    Ls = L - L4 * cos(p);
    H = L4 * sin(p);
    Hs = z + H;
    L23 = sqrt(Hs^2 + Ls^2);
    tau = acos((L2^2 + L3^2 + L23^2)/(2 * L2 * L3));
    s(3) = pi - tau;
    alpha = atan2(Hs, Ls);
    betta = acos((L23^2 + L2^2 - L3^2)/(2 * L23 * L2));
    s(2) = alpha + betta;
    L34 = L - L2 * cos(s(2));
    H34 = L2 * sin(s(2)) - z;
    Q34 = sqrt(L34^2 + H34^2);
    psi = acos((L4^2 + L3^2 - Q34^2)/(2 * L4 + L3));
    s(4) = pi - psi;
    
    in = 'axis';
    q = input(in);
    if q == 8 
        x = x + 0.05;
    elseif q == 2 
        x = x - 0.05;
    elseif q == 6 
        y = y + 0.05;
    elseif q == 4 
        y = y - 0.05;
    elseif q ==  9
        z = z + 0.05;
    elseif q == 7 
        z = z - 0.05;
    elseif q == 1 
        p = p + 0.05;
    elseif q == 3 
        p = p - 0.05;
    elseif q == 5 
        r = r + 0.05;
    elseif q == 0 
        r = r - 0.05;
    end
    
    show(robot,s)
end

%%
%show(robot,qSol)
%%
% s = [0 0 0 0 0];
% X = posX;
% Y = posY;
% Z = posZ;
% 
% s(5) = rotY
% s(1) = - atan(X/Y)
% L = sqrt(posX^2 + posY^2)
% L4s = L4 * cos(rotX)
% Ls = L - L4s
% H = posZ - L1 - L4 * sin(rotX)
% betta = atan(H/Ls)
% s(4) = betta + rotX
% L23 = sqrt(Ls^2 + H^2)
% alpha = acos((L2^2 + L3^2 - L23^2)/(2 * L2 * L3))
% s(3) = pi - alpha
% gamma = acos((L2^2 + L23^2 -L3^2)/(2 * L2 * L23))
% s(2) = gamma + betta
% qSol
% s
%%



