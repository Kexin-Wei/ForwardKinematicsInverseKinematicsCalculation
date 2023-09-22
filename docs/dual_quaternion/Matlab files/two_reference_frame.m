%% Desired trajectory in ECI
clc;clear;close all
% ECI
startpoint=[0,0,0];
xI_I=[1,0,0];
zI_I=[0,0,1];
fig1=figure();
plotrffm(startpoint,xI_I,zI_I,['k','k','k']);
hold on
axis equal

% ==== Initial State =====
% target satellite
E=0.75;
A=4*2.66; %10*7m
B=A*sqrt(1-E^2);
C=E*A;
wT=1;
wTE_E=[ 0,     0,  wT]; % roration
rTE_E=[ A+C,     0,   0];
vTE_E=[ 0,  B*wT,   0];
% r=[a*cos(w*t);b*sin(w*t);zeros(size(t))];
% v=[-a*w*sin(w*t),b*w*cos(w*t), 0];
% a=[-a*w*w*cos(w*t),-b*w*w*sin(w*t),0];

% t frame quaternion
qTE=QFromEuler(0,[0,0,1]); 

qEI=QFromEuler(pi/4,[0,1,0]); %inclination
rEI=[0,0,0];
dqEI=DQFromQTvec(qEI,rEI);

% desired reference frame
R=4; % radius
wD=wT;
wDT_T=[ 0,   0,       wD]; % roration
rDT_T=[ 0,   R,        0];
vDT_T=[ 0,   0,     R*wD];
% r=[0,R*cos(w*t), R*sin(w*t)]; % trajectory
% v=[0,-R*w*sin(w*t),R*w*cos(w*t)]; % velocity

% d frame quaternion
q1=QFromEuler(pi/2,[1,0,0]);
q2=QFromEuler(pi/2,[0,1,0]);
qDT=q1*q2;

%% Motion Define
dt = 0.01;
T = 2*pi/wTE_E(3);
t_set = 0:dt:T;

rt = zeros(3,size(t_set,2));
rd = zeros(3,size(t_set,2));

frameflag = 1;
for i=1:size(t_set,2)
    t=t_set(i);
    
    % new a aa
    [aTE_E,aaTE_E]=Tmotion(t,wT,A,B);   
    [aDT_T,aaDT_T]=Dmotion(t,wD,R); 
    
    % new reference frame in t
    TE_E = ReferFrame(rTE_E,wTE_E,vTE_E,aTE_E,aaTE_E,qTE);    
    DT_T = ReferFrame(rDT_T,wDT_T,vDT_T,aDT_T,aaDT_T,qDT);
    
    %frame T in frame I        
    rTI_I_dq = dqEI * TE_E.rdq * dqEI.conj;
    rTI_I_v  = rTI_I_dq.v(2:4);
    
    % frame D in frame I
    dqTI = dqEI * TE_E.dq;
    rDT_I_dq = dqTI * DT_T.rdq * dqTI.conj;
    rDT_I_v=rDT_I_dq.v(2:4); % x axis vector
    
    rDI_I_dq = rDT_I_dq + rTI_I_dq;
    rDI_I_v  = rDI_I_dq.v(2:4); % reference frame origin
    
    if mod(i,1/dt/4)==1 && frameflag
    %if i==400
        % frame T
        xT_I = rTI_I_v/norm(rTI_I_v);
        zI_I_dq = DQFromVec(zI_I);
        zT_I_dq = dqTI *zI_I_dq *dqTI.conj;
        zT_I_v  = zT_I_dq.v(2:4);
        plotrffm(rTI_I_v,xT_I,zT_I_v,['r','g','b']);
        
        % frame D
        xD_I=rDT_I_v/norm(rDT_I_v);
        dqDI=dqTI*DT_T.dq;
        zD_I_dq=dqDI*zI_I_dq*dqDI.conj;
        zD_I_v=zD_I_dq.v(2:4);
        plotrffm(rDI_I_v,xD_I,zD_I_v,['m','y','c']);
    end
    % next t Frame T to E
    vTE_E=vTE_E+aTE_E*dt;
    wTE_E=wTE_E+aaTE_E*dt;
    rTE_E=rTE_E+vTE_E*dt;
    
    delta_dqTE=(TE_E.wdq*TE_E.dq)*(1/2);
    delta_qTE=delta_dqTE.real;
    qTE=qTE+delta_qTE*dt;
    
    % next t frame D to T
    vDT_T=vDT_T+aDT_T*dt;
    wDT_T=wDT_T+aaDT_T*dt;
    rDT_T=rDT_T+vDT_T*dt;
    
    delta_dqDT=(DT_T.wdq*DT_T.dq)*(1/2);
    delta_qDT=delta_dqDT.real;
    qDT=qDT+delta_qDT*dt;    
    
    rt(:,i)=rTI_I_v;
    rd(:,i)=rDI_I_v;
end
view([20,30])
%% trajectory
fig2=figure();
plot3(rt(1,:),rt(2,:),rt(3,:),'k')
hold on
grid on
view([45,45])
xlabel('x')
ylabel('y')
axis equal
plot3(rd(1,:),rd(2,:),rd(3,:),'r')
legend('Target','Desired')

%% inside function
function [a,aa]=Tmotion(t,w,a,b)
    a=[-a*w*w*cos(w*t),-b*w*w*sin(w*t),0]; % linear accelaration
    aa=zeros(1,3);
end

function [a,aa]=Dmotion(t,w,R)
    a=[0,-R*w*w*cos(w*t),-R*w*w*sin(w*t)]; % linear accelaration
    aa=zeros(1,3);
end