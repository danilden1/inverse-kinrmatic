clear all;
close all;
clc;



%%
% L1 = 0.2;
% L2 = 0.3;
% L3 = 0.3;
% L4 = 0.05;
% L5 = 0.1;

syms L1 L2 L3 Lg
a = [0 L2 L3 0 0 0];
d = [-L1 0 0 0 Lg 0];

theta = [pi/2  0 0 -pi/2 0 pi/2];
alpha = [-pi/2 0 0 -pi/2 0 pi/2];
trigalpha = [cos(alpha)
    sin(alpha)
    -cos(alpha)
    -sin(alpha)]
%%
s = theta;
al = alpha;

x = 0.0;
y = 0.75;
z = 0;
p = 0;
r = 0;

syms nx ny nz ox oy oz ax ay az px py pz;
syms s1 s2 s3 s4 s5;


stheta = [s1 s2 s3 s4 s5];

T06 = [nx ox ax px
        ny oy ay py
        nz oz az pz
        0 0 0 0]

offset = [0 0 0 0 0 0];
%a = [0 8 0 0 0 0]; % in inches
%d = [13 0 2.5 8 0 2.5]; % in inches
type = ['r','r','r','r','r','r'];
base = [0; 0; 0];

% See http://medesign.seas.upenn.edu/index.php/Courses/MEAM520-12C-P01-IK


i = 1
A01 = [cos(stheta(i)) 0*sin(stheta(i)) -1*sin(stheta(i)) a(i)*cos(stheta(i))
        sin(stheta(i)) 0*cos(stheta(i)) 1*cos(stheta(i)) a(i)*sin(stheta(i))
        0 -1 0 d(i)
        0 0 0 1];
i = 2
A12 = [cos(stheta(i)) -1*sin(stheta(i)) 0*sin(stheta(i)) a(i)*cos(stheta(i))
        sin(stheta(i)) 1*cos(stheta(i)) 0*cos(stheta(i)) a(i)*sin(stheta(i))
        0 0 1 d(i)
        0 0 0 1];
T = A01*A12
i = 3
A23 = [cos(stheta(i)) -1*sin(stheta(i)) 0*sin(stheta(i)) a(i)*cos(stheta(i))
        sin(stheta(i)) 1*cos(stheta(i)) 0*cos(stheta(i)) a(i)*sin(stheta(i))
        0 0 1 d(i)
        0 0 0 1];
T = A01*A12*A23
i = 4
A34 = [cos(stheta(i)) 0*sin(stheta(i)) -1*sin(stheta(i)) a(i)*cos(stheta(i))
        sin(stheta(i)) 0*cos(stheta(i)) 1*cos(stheta(i)) a(i)*sin(stheta(i))
        0 -1 0 d(i)
        0 0 0 1];
T = A01*A12*A23*A34
i = 5
A45 =[cos(stheta(i)) -1*sin(stheta(i)) 0*sin(stheta(i)) a(i)*cos(stheta(i))
        sin(stheta(i)) 1*cos(stheta(i)) 0*cos(stheta(i)) a(i)*sin(stheta(i))
        0 0 1 d(i)
        0 0 0 1];
T = A01*A12*A23*A34*A45 
i = 6
A56 = [0 0 1 0
        1 0 0 0
        0 1 0 0
        0 0 0 1];
%%

%%T = A01*A12*A23*A34*A45*A56

nx == sin(s1)*sin(s5) + cos(s2 + s3 + s4)*cos(s1)*cos(s5);
ny == cos(s2 + s3 + s4)*cos(s5)*sin(s1) - cos(s1)*sin(s5);
nz == -sin(s2 + s3 + s4)*cos(s5);
ox == cos(s5)*sin(s1) - cos(s2 + s3 + s4)*cos(s1)*sin(s5);
oy == - cos(s1)*cos(s5) - cos(s2 + s3 + s4)*sin(s1)*sin(s5);
oz == sin(s2 + s3 + s4)*sin(s5);
ax == -sin(s2 + s3 + s4)*cos(s1);
ay == -sin(s2 + s3 + s4)*sin(s1);
az == -cos(s2 + s3 + s4);
px == cos(s1)*(L3*cos(s2 + s3) + L2*cos(s2) - Lg*sin(s2 + s3 + s4));
py == sin(s1)*(L3*cos(s2 + s3) + L2*cos(s2) - Lg*sin(s2 + s3 + s4));
pz == - L1 - L3*sin(s2 + s3) - L2*sin(s2) - Lg*cos(s2 + s3 + s4);


T06 = [nx ox ax px
        ny oy ay py
        nz oz az pz
        0 0 0 1]
    
eqns = [nx == sin(s1)*sin(s5) + cos(s2 + s3 + s4)*cos(s1)*cos(s5),
        ox == cos(s5)*sin(s1) - cos(s2 + s3 + s4)*cos(s1)*sin(s5),        
        ax == -sin(s2 + s3 + s4)*cos(s1),
        px == cos(s1)*(L3*cos(s2 + s3) + L2*cos(s2) - Lg*sin(s2 + s3 + s4)),
        ny == cos(s2 + s3 + s4)*cos(s5)*sin(s1) - cos(s1)*sin(s5),
        oy == - cos(s1)*cos(s5) - cos(s2 + s3 + s4)*sin(s1)*sin(s5),
        ay == -sin(s2 + s3 + s4)*sin(s1),
        py == sin(s1)*(L3*cos(s2 + s3) + L2*cos(s2) - Lg*sin(s2 + s3 + s4)),
        nz == -sin(s2 + s3 + s4)*cos(s5),
        oz == sin(s2 + s3 + s4)*sin(s5),
        az == -cos(s2 + s3 + s4),
        pz == - L1 - L3*sin(s2 + s3) - L2*sin(s2) - Lg*cos(s2 + s3 + s4),
];


vars = [s1 s2 s3 s4 s5]
S = solve(eqns,vars);
S
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

