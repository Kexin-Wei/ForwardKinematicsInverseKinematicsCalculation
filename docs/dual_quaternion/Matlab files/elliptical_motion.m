%% elliptical motion
clc;clear;close all
startpoint=[0,0,0];
xI_I=[1,0,0];
zI_I=[0,0,1];
fig1=figure();
plotrffm(startpoint,xI_I,zI_I,0);
hold on
axis equal

% Initial State
E=0.74;
A=2*2.66; %10*7m
B=A*sqrt(1-E^2);
C=E*A;
w=1;
wTE_E=[ 0,   0, w]; % roration
rTE_E=[ A,   0,   0];
vTE_E=[ 0, B*w,   0];
% r=[a*cos(w*t);b*sin(w*t)-c;zeros(size(t))];
% v=[-a*w*sin(w*t),b*w*cos(w*t), 0];
% a=[-a*w*w*cos(w*t),-b*w*w*sin(w*t),0];


% quaternion
qTE=QFromEuler(0,[0,0,1]);

qEI=QFromEuler(pi/4,[0,1,0]);
rEI=[0,0,0];
dqEI=DQFromQTvec(qEI,rEI);
%% Motion Define
dt=0.01;
T=2*pi/w;
t_set=0:dt:T;

rc=zeros(3,size(t_set,2));

frameflag=1;
for i=1:size(t_set,2)

    t=t_set(i);
    
    [aTE,aaTE]=motion(t,w,A,B);   

    TE_E=ReferFrame(rTE_E,wTE_E,vTE_E,aTE,aaTE,qTE);
    
    %frame T in frame I
    rTE_I_dq=dqEI*TE_E.rdq*dqEI.conj;
    rEI_I_dq=DQFromVec(rEI);
    rTI_I_dq=rTE_I_dq+rEI_I_dq;
    rTI_v=rTI_I_dq.v(2:4);
    
    if mod(i,1/dt/2)==1 && frameflag
        xT=rTI_v/norm(rTI_v);
        zI_dq=DQFromVec(zI_I);
        dqTI=dqEI*TE_E.dq;
        zT_dq=dqTI*zI_dq*dqTI.conj;
        zT_v=zT_dq.v(2:4);
        plotrffm(rTI_v,xT,zT_v,0); 
    end
    % next t
    vTE_E=vTE_E+aTE*dt;
    wTE_E=wTE_E+aaTE*dt;
    rTE_E=rTE_E+vTE_E*dt;

    rc(:,i)=rTI_v;
    
    delta_dqTE=(TE_E.wdq*TE_E.dq)*(1/2);
    delta_qTE=delta_dqTE.real;
    qTE=qTE+delta_qTE*dt;    
end

view([0,0])
saveas(fig1,"inclination.png")
view([45,45])
saveas(fig1,"trajectory.png")
%% compare
fig2=figure();
r=[A*cos(w*t_set);B*sin(w*t_set);zeros(size(t_set))]';
r_o=zeros(size(r));
for i=1:size(r,1)
    r_o(i,:)=vecrotate(r(i,:),qEI);
end
plot3(r_o(:,1),r_o(:,2),r_o(:,3),'r')
hold on
grid on
view([45,45])
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
plot3(rc(1,:),rc(2,:),rc(3,:),'k--')
legend('Original','Cumlative')

%% inside function
function [a,aa]=motion(t,w,a,b)
    a=[-a*w*w*cos(w*t),-b*w*w*sin(w*t),0]; % linear accelaration
    aa=zeros(1,3);
end
function rv_D=vecrotate(v_I,qDI)
    v_I_q=QFromVec(v_I);
    rv_D_q=qDI*v_I_q*qDI.conj;
    rv_D=rv_D_q.v(2:4);
end