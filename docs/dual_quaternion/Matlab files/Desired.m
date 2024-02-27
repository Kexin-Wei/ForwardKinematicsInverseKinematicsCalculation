function DT=Desired(t)
q1=QFromEuler(-pi/2,[0,0,1]); %z -90
q2=QFromEuler( pi/2,[1,0,0]); %x 90
qDT=q1*q2;

G=6.67428e-11;
M=5.9742e24;
mu=G*M;

A=2.66e7; %10*7m;
n=sqrt(mu/A);

R=20;

if t<400
    vDT=[0,0.025,0];
    wDT=[0,0,0];
    rDT=[ 0, -30, 0]+vDT*t;
else
    t=t-400;
    vDT=[0, R*n*sin(n*t),-R*n*cos(n*t)];
    wDT=[-n,0,0];
    rDT=[0, -R*cos(n*t),-20*sin(n*t)];
%     vDT=[0,0,0];
%     wDT=[0,0,0];
%     rDT=[ 0, -20, 0];
end
DT=SimpleReferFrame(rDT,vDT,wDT,qDT);
end