%% dual quaternion transformation
clc;clear;close all
%%
startpoint=[0,0,0];
xI=[1,0,0];
zI=[0,0,1];
fig1=figure();
plotrffm(startpoint,xI,zI,0);
hold on
axis equal

% Initial State
R=4; % radius
w=[ 0,   0, 1]; % roration
r=[ R,   0,   0];
v=[ 0, R*w(3),   0];
% r=[R*cos(w*t), R*sin(w*t),0]; % trajectory
% v=[-R*w*sin(w*t),R*w*cos(w*t),0]; % velocity

qBI=QFromEuler(0,[0,0,1]);

%% Motion Define
dt=0.01;
T=2*pi/w(3);
t_set=0:dt:T;

rc=zeros(3,size(t_set,2));

for i=1:size(t_set,2)

    t=t_set(i);
    
    [a,aa]=motion(t,w(3),R);   

    BI=ReferFrame(r,w,v,a,aa,qBI);
    
    if mod(i,1/dt/2)==1
        xB=r/norm(r);
        zI_dq=DQFromVec(zI);
        zB_dq=BI.dq*zI_dq*BI.dq.conj;
        zB=zB_dq.v(2:4);
        plotrffm(r,xB,zB,0); 
    end
    % next t
    v=v+a*dt;
    w=w+aa*dt;
    r=r+v*dt;

    rc(:,i)=r;
    
    delta_dqBI=(BI.wdq*BI.dq)*(1/2);
    delta_qBI=delta_dqBI.real;
    qBI=qBI+delta_qBI*dt;
    
end
%% compare
fig2=figure();
plot3(R*cos(w(3)*t_set),R*sin(w(3)*t_set),zeros(size(t_set)),'r')
hold on
grid on
view(2)
xlabel('x')
ylabel('y')
axis equal
plot3(rc(1,:),rc(2,:),rc(3,:),'k--')
legend('Original','Cumlative')

%% inside function
function [a,aa]=motion(t,w,R)
    a=[-R*w*w*cos(w*t),-R*w*w*sin(w*t),0]; % linear accelaration
    aa=zeros(1,3);
end