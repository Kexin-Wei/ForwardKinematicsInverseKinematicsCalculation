%% YZ plane circular motion with 2 phases
clc;clear;close all
startpoint=[0,0,0];
xT=[1,0,0];
zT=[0,0,1];
fig1=figure();
plotrffm_s(startpoint,xT,zT,5,0);
hold on
axis equal
xlim([-35 35])
ylim([-35 35])
zlim([-35 35])
view([45 45])


% Initial State
G=6.67428e-11;
M=5.9742e24;
mu=G*M;

A=2.66e7; %10*7m;

R=20; % radius
n=sqrt(mu/A);
rDT=[ 0, -30, 0];

q1=QFromEuler(-pi/2,[0,0,1]); %z -90
q2=QFromEuler( pi/2,[1,0,0]); %x 90
qDT=q1*q2;
%% Motion Define
% motion 1
dt1=1e-2;
T1=400;
t_set1=0:dt1:T1;
n_t1=size(t_set1,2);

rd1=zeros(3,n_t1);

frameflag1=1;
for i=1:n_t1

    t=t_set1(i);
    
    [vDT,wDT]=motion1(t);
        
    DT = SimpleReferFrame(rDT,vDT,wDT,qDT);
    
    if mod(i,1e4)==1 && frameflag1
        xD=rDT/norm(rDT);
        zI_dq=DQFromVec(zT);
        zD_dq=DT.dq*zI_dq*DT.dq.conj;
        zD=zD_dq.v(2:4);
        plotrffm_s(rDT,xD,zD,5,0); 
    end
    % next t
    rDT=rDT+vDT*dt1;

    rd1(:,i)=rDT;
    
    delta_dqDT=(DT.wdq*DT.dq)*(1/2);
    delta_qDT=delta_dqDT.real;
    qDT=qDT+delta_qDT*dt1;    
end

%% motion 2
dt2=1e-2;
T2=500-T1;
t_set2=0:dt2:T2;
n_t2=size(t_set2,2);

rd2=zeros(3,n_t2);
frameflag2=1;
for i=1:n_t2

    t=t_set2(i);
    
    [vDT,wDT]=motion2(t,1,R);
   
    DT = SimpleReferFrame(rDT,vDT,wDT,qDT);
    
    if mod(i,8e2)==1 && frameflag2
        xD=rDT/norm(rDT);
        zI_dq=DQFromVec(zT);
        zD_dq=DT.dq*zI_dq*DT.dq.conj;
        zD=zD_dq.v(2:4);
        plotrffm_s(rDT,xD,zD,5,0); 
    end
    % next t
    rDT=rDT+vDT*dt2;

    rd2(:,i)=rDT;
    
    delta_dqDT=(DT.wdq*DT.dq)*(1/2);
    delta_qDT=delta_dqDT.real;
    qDT=qDT+delta_qDT*dt2;    
end

%% Trajectory
fig2=figure();
plotrffm(startpoint,xT,zT,0);
hold on
grid on
view([90,0])
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
rd=[rd1,rd2];
plot3(rd(1,:),rd(2,:),rd(3,:),'r--')
%legend('Original','Cumlative')

%% inside function
function [v,w]=motion1(t)
    v=[0,0.025,0];
    w=[0,0,0];
end
function [v,w]=motion2(t,n,R)
    v=[0, R*n*sin(n*t),-R*n*cos(n*t)];
    w=[-n,0,0];
end