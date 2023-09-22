% molynia orbit with reference frame
clc;clear;close all

% origin
xI=[1,0,0];
zI=[0,0,1];
plotrffm_s([0,0,0],xI,zI,5e6,0);
hold on
grid on

% molniya
G=6.67428e-11;
M=5.9742e24;
mu=G*M;

%coe
E=0.74;
A=2.66e7; %10*7m
B=A*sqrt(1-E^2);
C=E*A;

RAAN=329.6*pi/180;
incl=63.4*pi/180;
aop=270*pi/180;

% initial state
rTI=[ A+C,   0,   0];
vp=sqrt(2*mu/rTI(1)-mu/A);
vTI=[0,vp,0];
h=cross(rTI,vTI);

ta=1; % for TA
dt=0.01;
T=2*pi/ta;
t_set=0:dt:T;

rc=zeros(size(t_set,2),3);
vc=zeros(size(t_set,2),3);

for i=1:size(t_set,2)
    t=t_set(i);    
    TA=ta*t;
    coe=[h(3),E,RAAN,incl,aop,TA];
    [rTI,vTI,qTI]=sv_from_coe(coe,mu);
    rc(i,:)=rTI;
    vc(i,:)=vTI;
    
    if mod(i,1/dt/4)==1
        xT=rTI/norm(rTI);
        
        zIq=QFromVec(zI);
        zTq=qTI*zIq*qTI.conj;
        zT=zTq.v(2:4); %zT=cross(rTI,vTI); zT=zT/norm(zT);
        plotrffm_s(rTI,xT,zT,5e6,0);
    end
    
end
axis equal
view([-60 20])
%% trajectory
plot3(rc(:,1),rc(:,2),rc(:,3),'r')

%% RAAN and TA change in Molniya orbit

J2=0.0010826267;
Re=6378.137*1e3;

constant=-3/2*sqrt(mu)*J2*Re/(1-E^2)^2/A^(7/2);
dRaan=constant*cos(incl)
dTa=constant*(5*(sin(incl)^2)/2-2)
