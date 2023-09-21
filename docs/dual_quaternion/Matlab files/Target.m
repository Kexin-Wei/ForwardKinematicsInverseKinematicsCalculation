function [rTI,vTI,wTI,qTI]=Target(TA)
% molniya
G=6.67428e-11;
M=5.9742e24;
mu=G*M;

%coe
E=0.7;
A=2.66e7; %10*7m
C=E*A;

RAAN=329.6*pi/180;
incl=63.4*pi/180;
aop=270*pi/180;

% h from perigee
rTI=[ A+C,   0,   0];
vp=sqrt(2*mu/rTI(1)-mu/A);
vTI=[0,vp,0];
h=cross(rTI,vTI);

coe=[h(3),E,RAAN,incl,aop,TA];
[rTI,vTI,qTI]=sv_from_coe(coe,mu);

wTI= cross(rTI,vTI)/(norm(rTI)*norm(rTI));
end