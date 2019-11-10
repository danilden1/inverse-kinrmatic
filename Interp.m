clear all;
close all;
clc;

g = ncgr_graphic();

%%
l1 = 0.2;
l2 = 0.3;
l3 = 0.3;
l4 = 0.15;
l5 = 0.1;

L1 = 0.2;
L2 = 0.3;
L3 = 0.3;
L4 = 0.05;
L5 = 0.1;

global N_DOFS;
N_DOFS = 6;

a = [0 L2 L3 0 0 0];
d = [-L1 0 0 0 L5+L4 0];
al = a;
t = [pi/2  0 0 -pi/2 0 pi/2];
theta = t;
alpha = [-pi/2 0 0 -pi/2 0 pi/2];
offset = [0 0 0 0 0 0];

type = ['r','r','r','r','r','r'];
base = [0; 0; 0];

% See http://medesign.seas.upenn.edu/index.php/Courses/MEAM520-12C-P01-IK
lb = [deg2rad(-360); deg2rad(-360); deg2rad(-360); deg2rad(-360); deg2rad(-360); deg2rad(-360)];
ub = [deg2rad(360); deg2rad(360); deg2rad(360); deg2rad(360); deg2rad(360);  deg2rad(360)];
puma = cgr_create(t, d, a, alpha, offset, type, base, ub, lb);
    puma = cgr_self_update(puma, t);
    g = ncgr_plot(g, puma);

i = 0;
s = [-pi/2 : 0.5 : pi/2]
%%
rad2deg(s)
for i1  = 1 : 7
    for i2  = 1 : 7
        for i3  = 1 : 7
            for i4  = 1 : 7

                    theta(1) = t(1) + s(i1);
                    theta(2) = t(2) + s(i2);
                    theta(3) = t(3) + s(i3);
                    theta(4) = t(4) + s(i4);
                    theta
                    i = 1;
A01 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
        sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
        0 sin(al(i)) cos(al(i)) d(i)
        0 0 0 1];
i = 2;
A12 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
        sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
        0 sin(al(i)) cos(al(i)) d(i)
        0 0 0 1];
i = 3;
A23 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
        sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
        0 sin(al(i)) cos(al(i)) d(i)
        0 0 0 1];
i = 4;
A34 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
        sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
        0 sin(al(i)) cos(al(i)) d(i)
        0 0 0 1];
i = 5;
A45 =[cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
        sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
        0 sin(al(i)) cos(al(i)) d(i)
        0 0 0 1];
i = 6;
A56 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
        sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
        0 sin(al(i)) cos(al(i)) d(i)
        0 0 0 1];
%%

T = A01*A12*A23*A34*A45*A56

    puma = cgr_self_update(puma, theta);
    g = ncgr_plot(g, puma);

                end
            end
        end
end