clear all;
close all;
clc;



%%
l1 = 20;
l2 = 30;
l3 = 30;
l4 = 15;
l5 = 0.1;


a = [0 l2 l3 0 0 0];
d = [-l1 0 0 0 l4 0];

theta = [pi/2  0 0 -pi/2 0 pi/2];
alpha = [-pi/2 0 0 -pi/2 0 pi/2];

s = theta;
al = alpha;

x = 0.0;
y = 75;
z = 0;
p = 0;
r = 0;

g = ncgr_graphic();
global N_DOFS;
N_DOFS = 6;
offset = [0 0 0 0 0 0];
%a = [0 8 0 0 0 0]; % in inches
%d = [13 0 2.5 8 0 2.5]; % in inches
type = ['r','r','r','r','r','r'];
base = [0; 0; 0];

% See http://medesign.seas.upenn.edu/index.php/Courses/MEAM520-12C-P01-IK
lb = [deg2rad(-360); deg2rad(-360); deg2rad(-360); deg2rad(-360); deg2rad(-360); deg2rad(1)];
ub = [deg2rad(360); deg2rad(360); deg2rad(360); deg2rad(360); deg2rad(360); deg2rad(-1)];
puma = cgr_create(theta, d, a, alpha, offset, type, base, ub, lb);
    puma = cgr_self_update(puma, theta);
    g = ncgr_plot(g, puma);

while 1
       in = 'axis';
    q = input(in);
    if q == 8 
        x = x + 0.5;
    elseif q == 2 
        x = x - 0.5;
    elseif q == 6 
        y = y + 0.5;
    elseif q == 4 
        y = y - 0.5;
    elseif q ==  9
        z = z + 0.5;
    elseif q == 7 
        z = z - 0.5;
    elseif q == 1 
        p = p + 0.05;
    elseif q == 3 
        p = p - 0.05;
    elseif q == 5 
        r = r + 0.05;
    elseif q == 0 
        r = r - 0.05;
    end
    
    pos1 = [x, y, z, r, p]
    s = [0 0 0 0 0];
    s(5) = r
    
    L = sqrt(x^2 + y^2)
    %s(1) = acos((x^2 + L^2 - y^2)/(2 * x * L))
    s(1) = atan2(x, y)
    L23 = L - l4 * cos(p)
    H23 = z - l4 * sin(p)
    Q23 = sqrt(H23^2 + L23^2)
    alpha = acos((L23^2 + Q23^2 - H23^2)/(2 * L23 * Q23))
    betta = acos((Q23^2 + l2^2 - l3^2)/(2 * Q23 * l2))
     if H23 <= 0
       s(2) = -alpha - betta
    else 
        s(2) =  -betta + alpha
    end
    
    
    H34 = l2 * sin(s(2)) - z
    L34 = L - l2 * cos(s(2))
    Q34 = sqrt(L34^2 + H34^2)
    psi = acos((l3^2 + l4^2 - Q34^2)/(2 * l3 * l4))
        s(4) = pi - psi

    gamma = acos((l2^2 + l3^2 - Q23^2)/(2 * l2 * l3))
    s(3) = pi - gamma

s
rad2deg(s)

theta(1) = s(1) + pi/2;
theta(2) = -s(2);
theta(3) = -s(3);
theta(4) = s(4) - pi / 2;
theta(5) = s(5);
theta(6) = pi/2;
rad2deg(theta)
% i = 1
% A01 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
%         sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
%         0 sin(al(i)) cos(al(i)) d(i)
%         0 0 0 1];
% i = 2
% A12 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
%         sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
%         0 sin(al(i)) cos(al(i)) d(i)
%         0 0 0 1];
% T = A01*A12
% i = 3
% A23 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
%         sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
%         0 sin(al(i)) cos(al(i)) d(i)
%         0 0 0 1];
% T = A01*A12*A23
% i = 4
% A34 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
%         sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
%         0 sin(al(i)) cos(al(i)) d(i)
%         0 0 0 1];
% T = A01*A12*A23*A34
% i = 5
% A45 =[cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
%         sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
%         0 sin(al(i)) cos(al(i)) d(i)
%         0 0 0 1];
% T = A01*A12*A23*A34*A45 
% i = 6
% A56 = [cos(theta(i)) -cos(al(i))*sin(theta(i)) sin(al(i))*sin(theta(i)) a(i)*cos(theta(i))
%         sin(theta(i)) cos(al(i))*cos(theta(i)) -sin(al(i))*cos(theta(i)) a(i)*sin(theta(i))
%         0 sin(al(i)) cos(al(i)) d(i)
%         0 0 0 1];
% %%
% 
% T = A01*A12*A23*A34*A45*A56
% 
% 
% start = [T(1,1) T(1,2) T(1,3) 
%             T(2,1) T(2,2) T(2,3) 
%             T(3,1) T(3,2) T(3,3)];
% eul = rotm2eul(start);


   % puma = cgr_create(theta, d, a, alpha, offset, type, base, ub, lb);
    puma = cgr_self_update(puma, theta);
    g = ncgr_plot(g, puma);
    rad2deg(theta)
    

%pos2 = [T(1, 4) T(2, 4) T(3, 4) eul]

end
% 
% nx = T(1, 1)
% ny = T(2, 1)
% nz = T(3, 1)
% 
% ox = T(1, 2)
% oy = T(2, 2)
% oz = T(3, 2)
% 
% ax = T(1, 3)
% ay = T(2, 3)
% az = T(3, 3)
% 
% px = T(1, 4)
% py = T(2, 4)
% pz = T(3, 4)
% 

