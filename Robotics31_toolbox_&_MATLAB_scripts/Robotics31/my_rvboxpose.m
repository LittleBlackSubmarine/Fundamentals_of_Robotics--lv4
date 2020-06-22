function TA0 = my_rvboxpose(P,f,TC0,zC,c)

uc = 320;    
vc = 240;
fi_ = [];  % Angles array
ro_ = [];  % Distances array
p = [];    % Edges intersections coordinates


for i=1:4
     fi_ = [fi_ P(1,i)];
     ro_ = [ro_ P(2,i)];
end

% Solving coordinates of lines(edges) intersections
for i = 1:2
    for j = 3:4
                
syms uk vk 'real';
eqn1 = (uk-uc)*cos(fi_(i)) + (vk - vc)*sin(fi_(i)) == ro_(i);
eqn2 = (uk-uc)*cos(fi_(j)) + (vk - vc)*sin(fi_(j)) == ro_(j);
[uk, vk] = solve([eqn1, eqn2]);
p = [p [double(uk) double(vk)]'];

    end
end

% Finding coordinates of object center of mass
u =(p(1,1)+p(1,2)+p(1,3)+p(1,4))/4; 
v =(p(2,1)+p(2,2)+p(2,3)+p(2,4))/4;

% Translations
x = (u - uc)*((zC-c)/f);
y = (v - vc)*((zC-c)/f);
z = zC - c/2;

% Translation vector
tAC = [x y z]';

% Distances between detected edge s
% --> Bigger distance between edges -> shorter edges and vice versa
 
d1 = abs(double(ro_(1)) - double(ro_(2)));
d2 = abs(double(ro_(3)) - double(ro_(4)));

% Finding the angle of object
% --> x axis in direction of longer edge
if d1 > d2
    alpha = fi_(1);
else
    alpha = fi_(3);
end

% Object rotation matrix
RAC = [cos(alpha) -sin(alpha) 0;
       sin(alpha)  cos(alpha) 0;
           0            0     1];

% Translation matrix between object and camera
TAC = [RAC tAC;
       0 0 0 1];

% Translation matrix between object and manipulator base coordinate system
TA0 = TC0*TAC;
