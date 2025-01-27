% The following parameters must be defined before running function rvsim:
%
%     method     - image processing method: 'corners' or 'edges'. Default is
%                  'edges'.
%     xmin, xmax - working range, i.e. minimum and maximum x-coordinate 
%                  which can be reached by the tool 
%     zT0        - height of the working plane relative to the base
%                  c. s. of the robot


% table pose

TT0 = [eye(3) [0.5*(xmax + xmin); 0; zT0];
    0 0 0 1];

% box parameters

a = 10;
b = 30;
c = 10;

% camera parameters

w=640;
h=480;
f=1000;
uc=w/2;
vc=h/2;
r = sqrt(a^2+b^2)/2;
wb=10;
zC = (xmax - xmin + 2 * r) / (w - 2 * wb) * f + c;
TCT = [1 0 0 0;
    0 -1 0 0;
    0 0 -1 zC;
    0 0 0 1];
TC0 = TT0*TCT;

% random selection of the box pose

ymin = (wb-vc)*(zC-c)/f+r;
ymax = (h-wb-vc)*(zC-c)/f-r;
XA0 = [[xmin ymin]'+diag([xmax-xmin ymax-ymin])*rand(2,1); zT0; 1];
XAC = inv(TC0)*XA0;
alpha = -pi+rand(1)*2*pi;

% projection of the box to the camera image

I = imgbox(XAC,alpha,a,b,c,f,uc,vc,w,h);
%I = imgobjt(XAC,alpha,a,b,c,f,uc,vc,w,h);

figure(1),imshow(I,[0 1])
title('slika snimljena kamerom')

if method == 'corners'
    C = cornermetric(I,'SensitivityFactor', 0);
    ROI = C(3:h-3,3:w-3);
    C = zeros(h,w);
    C(3:h-3,3:w-3) = ROI;
    P = [];
    hws = 3;
    for i = 1:4
        [Y, V] = max(C);
        [y, u] = max(Y');
        v = V(u);
        P = [P [u v]'];
        C(v, u) = 0;
        C(max(v-hws,1):min(v+hws,h),max(u-hws,1):min(u+hws,w)) = zeros(min(v+hws,h)-max(v-hws,1)+1,min(u+hws,w)-max(u-hws,1)+1);
    end
    hold on
    plot(P(1,:)',P(2,:)','g+')
    hold off
    P = P - ones(2,4);
else
    % thresholding
    
    Ibw = im2bw(I,0.5);

    figure(2),imshow(Ibw,[0 1])
    title('binarna slika dobivena na temelju praga (thresholding)')

    pause
        
    % edge detection

    E = edge(I,'canny',[0.1 0.2]);
    figure(2),imshow(E,[0 1])
    title('Detekcija rubova')
    pause

    % hough transform

    nP = 4;
    [H, Phi, Rho, P, V] = hough2(E,[w h]'/2,pi/180,1,2,nP,10,10);
    figure(3),imshow(H/max(max(H)),[0 1])
    title('Houghova ravnina')
    xlabel('phi')
    ylabel('rho')
    pause

    figure(1),hold on,plot([V(1,:); V(3,:)],[V(2,:); V(4,:)],'g'),hold off
    title('pravci dobiveni Houghovom transformacijom')
    pause
end

% trajectory generation

[Qc, TA0est] = rvtraj(P,f,TT0,TCT,c);

Xe = XA0 - TA0est(:,4);
eX = sqrt(Xe'*Xe);

ca = cos(alpha);
sa = sin(alpha);

ealpha = acos(abs(ca*TA0est(1,1)+sa*TA0est(2,1)));

%alphaest = atan2(TA0est(2,1),TA0est(1,1));

%ealpha = alpha - alphaest;
%if ealpha > pi/2
%    ealpha = ealpha - pi;
%elseif ealpha < -pi/2
%    ealpha = ealpha + pi;
%end

disp(['pogreska estimacije: pozicija: ' num2str(eX)...
    ' mm,   orijentacija: ' num2str(ealpha*180/pi)...
    ' deg'])
disp('')

if eX > 20 || ealpha > 5*pi/180
    disp('PREVELIKA POGRESKA ESTIMACIJE POLOZAJA PREDMETA!!!')
    return
end

disp('ESTIMACIJA POLOZAJA PREDMETA JE DOVOLJNO TOCNA.')
%return;

% display scene

[robot, plotbox, azimuth, elevation] = createscene();

if isempty(robot)
    return
end

env.object(1) = cuboid(a, b, c);

TA0 = [rotz(alpha)*roty(pi) XA0(1:3) - [0 0 c/2]'; 0 0 0 1];

env.object(1).X = TA0 * env.object(1).X;

figure(4)

dyn3dscene(robot, Qc, env, plotbox, azimuth, elevation, 0.2, 0.001)

