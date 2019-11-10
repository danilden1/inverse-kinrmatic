clear all;
close all;
clc;



%%
l1 = 0.2;
l2 = 0.3;
l3 = 0.3;
l4 = 0.15;
l5 = 0.1;


a = [0 l2 l3 0 0 0];
d = [-l1 0 0 0 l4 0];

theta = [pi/2  0 0 -pi/2 0 pi/2];
alpha = [-pi/2 0 0 -pi/2 0 pi/2];

s = theta;
al = alpha;

x = 0.0;
y = 0.75;
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
lb = [deg2rad(-180); deg2rad(-75); deg2rad(-236); deg2rad(-580); deg2rad(-120); deg2rad(1)];
ub = [deg2rad(110); deg2rad(240); deg2rad(60); deg2rad(40); deg2rad(110); deg2rad(-1)];
puma = cgr_create(theta, d, a, alpha, offset, type, base, ub, lb);
    puma = cgr_self_update(puma, theta);
    g = ncgr_plot(g, puma);

while 1
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
    
    t = 0;
    pos1 = [x, y, z, r, p, t]
    eul = (r p t);
    rot = eul2rotm(eul);
    %T06 = T
%S = solve(T06)
% 
% 
% start = [T(1,1) T(1,2) T(1,3) 
%             T(2,1) T(2,2) T(2,3) 
%             T(3,1) T(3,2) T(3,3)];
% eul = rotm2eul(start);
% 
% 
%     rad2deg(theta)
%     
%pos2 = [T(1, 4) T(2, 4) T(3, 4) eul]

    
    
rad2deg(s)

theta(1) = s(1) + pi/2;
theta(2) = -s(2);
theta(3) = -s(3);
theta(4) = s(4) - pi / 2;
theta(5) = s(5);
theta(6) = pi/2;
rad2deg(theta)

   % puma = cgr_create(theta, d, a, alpha, offset, type, base, ub, lb);
    puma = cgr_self_update(puma, theta);
    g = ncgr_plot(g, puma);
    rad2deg(theta)
    

%pos2 = [T(1, 4) T(2, 4) T(3, 4) eul]

end