function [Qc, TA0est] = rvtraj(P,f,TP0,TCP,c)

% box pose estimation

TC0 = TP0*TCP;
TA0est = my_rvboxpose(P,f,TC0,TCP(3,4),c);

% Initial robot configuration

T60(:,:,1) = [1 0 0  -19.3474;
    0 1 0 0;
    0 0 1 196;
    0 0 0 1];

% approach configuration 

T6A = [1 0 0 0;
    0 1 0 0;
    0 0 1 15;
    0 0 0 1];

T60(:,:,2) = TA0est*T6A;

% goal configuration

T6A = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

T60(:,:,3) = TA0est*T6A;

% trajectory


m = 3;

Q = [];

for i = 1:m
    Q = [Q my_invkin(T60(:,:,i))];
    [m,n] = size(Q);
    
    if i > 1
        for j = 1:n
            if Q(j,i) - Q(j,i-1) > pi
                Q(j,i) = Q(j,i) - 2 * pi;
            else
                if Q(j,i) - Q(j,i-1) < -pi
                   Q(j,i) = Q(j,i) + 2 * pi;
                end
            end
        end
    end
end

Q = [Q; 50 50 50];

dqgr = [8 8 8 8 8 8 100]';
ddqgr = [16 16 16 16 16 16 10000]';

[Qc, dQc, ddQc] = hocook(Q, dqgr, ddqgr, 0.002);

nt = length(Qc);