%% Quaternion Rotation
clc;clear;close all

v = [1,1,1];
theta = pi/2;
n = [-1,-1,1];
q = QFromEuler(theta,n);
vq = QFromVec(v);

vq2=q*vq*q.conj;
v2=vq2.v(2:4);
norm(v2)


% verify
n_n=n/norm(n);
u=sin(theta/2)*n_n;
u1=u(1);u2=u(2);u3=u(3);u4=cos(theta/2);
A=[ u1^2-u2^2-u3^2+u4^2, 2*(u1*u2-u3*u4), 2*(u3*u1+u2*u4);
    2*(u1*u2+u3*u4), u2^2-u3^2-u1^2+u4^2, 2*(u2*u3-u1*u4);
    2*(u3*u1-u2*u4), 2*(u2*u3+u1*u4), u3^2-u1^2-u2^2+u4^2];
v2_v=A*v';


% plot
% original
x=[0,v(1)];
y=[0,v(2)];
z=[0,v(3)];
po=plot3(x,y,z,'b');
hold on 
grid on
xlabel('x');
ylabel('y');
zlabel('z');
scatter3(x(2),y(2),z(2),'b*')

% quaternion
x=[0,v2(1)];
y=[0,v2(2)];
z=[0,v2(3)];
pv2=plot3(x,y,z,'k');
scatter3(x(2),y(2),z(2),'k*')

% matrix
x=[0,v2_v(1)];
y=[0,v2_v(2)];
z=[0,v2_v(3)];
pv2_v=plot3(x,y,z,'g--');
scatter3(x(2),y(2),z(2),'g*')

legend([po,pv2,pv2_v],{'Original','Quaternion','Rotation Matrix'})