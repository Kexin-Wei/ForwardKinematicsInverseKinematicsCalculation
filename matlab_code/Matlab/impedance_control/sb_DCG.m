clc;clear;close all
% Parameters
syms m1 m2 m3 real
syms l1 l2 l3 real
syms g real
syms Ic1xx Ic1yy Ic1zz Ic2xx Ic2yy Ic2zz Ic3xx Ic3yy Ic3zz real
ms   = [m1, m2, m3];
ls   = [l1, l2, l3];
Ic1s = [Ic1xx, Ic1yy, Ic1zz];
Ic2s = [Ic2xx, Ic2yy, Ic2zz];
Ic3s = [Ic3xx, Ic3yy, Ic3zz];
% inertia tensor
Ic1 = diag([Ic1xx, Ic1yy, Ic1zz]);
Ic2 = diag([Ic2xx, Ic2yy, Ic2zz]);
Ic3 = diag([Ic3xx, Ic3yy, Ic3zz]);

% numerical 
msn = [3.92 2.16   1]; % mass(kg) m1 m2 m3 
lsn = [0.4  0.3  0.2]; % link length(m) l1 l2 l3
gn = 9.8; 
Ic1n = [0.0587, 0.0128, 0.0587]; % kg*meter^2
Ic2n = [0.0052, 0.0188, 0.0188];
Ic3n = [0.0016, 0.0042, 0.0042];
%% DH parameters
syms q1 q2 q3 real

dh = [q1, l1, -pi/2,   0;
      q2,  0,     0,  l2;
      q3,  0,     0, l3];

n_dof = size(dh,1);
Ts = cell(1,n_dof);
As = cell(1,n_dof);
T  = eye(4);

for i=1:n_dof
    A = dh_matrix(dh(i,:));
    T = T*A;
    As{i} = A;
    Ts{i} = T;
end
%% Kinetic Energy & Potential Energy
syms qd1 qd2 qd3 real
syms qdd1 qdd2 qdd3 real
n_dof = 3;
w00  = [0,0,0]';
v00  = [0,0,0]';
rc11 = [0,    l1/2,  0]';
rc22 = [-l2/2,   0,  0]';
rc33 = [-l3/2,   0,  0]';

% dof 1
R10  = As{1}(1:3,1:3)'; 
r01  = As{1}(1:3,4);
w11  = R10*(w00 + qd1*[0,0,1]');
v11  = R10*v00 + cross(w11,R10*r01);
vc1  = v11 + cross(w11,rc11);

% dof 2
R21  = As{2}(1:3,1:3)'; 
r12  = As{2}(1:3,4);
w22  = R21*(w11 + qd2*[0,0,1]');
v22  = R21*v11 + cross(w22,R21*r12);
vc2  = v22 + cross(w22,rc22);

% dof 3
R32  = As{3}(1:3,1:3)'; 
r23  = As{3}(1:3,4);
w33  = R32*(w22 + qd3*[0,0,1]');
v33  = R32*v22 + cross(w33,R32*r23);
vc3  = v33 + cross(w33,rc33);

% kinetic
T1 = 1/2*m1*vc1'*vc1 + 1/2*w11'*Ic1*w11;
T2 = 1/2*m2*vc2'*vc2 + 1/2*w22'*Ic2*w22;
T3 = 1/2*m3*vc3'*vc3 + 1/2*w33'*Ic3*w33;
T  = T1 + T2 + T3;
% potential 
p00 = [0,0,0]';
p01 = Ts{1}(1:3,4);
p02 = Ts{2}(1:3,4);
p03 = Ts{3}(1:3,4);
pc1 = (p00+p01)/2;
pc2 = (p01+p02)/2;
pc3 = (p02+p03)/2;
P   = m1*g*pc1(3) + m2*g*pc2(3) + m3*g*pc3(3);
%% Lagrange Calc
L = simplify(T - P);

% \frac{\partial L}{\partial qd} [3x1]
e1 = diff(L,qd1);
e2 = diff(L,qd2);
e3 = diff(L,qd3);

% \frac{d}{dt}\frac{\partial L}{\partial qd}
% chain rule d/dt = \partial L / \partial q1 * d q1/dt +...
m11 = diff(e1,qd1); m12 = diff(e1,qd2); m13 = diff(e1,qd3);
m21 = diff(e2,qd1); m22 = diff(e2,qd2); m23 = diff(e2,qd3);
m31 = diff(e3,qd1); m32 = diff(e3,qd2); m33 = diff(e3,qd3);
M = [m11, m12, m13;
     m21, m22, m23;
     m31, m32, m33];
M = simplify(M);

% C matrix
pM1pq = jacobian(M(:,1),[q1,q2,q3]);
pMpq1 = diff(M,q1);
C1 = 1/2*(pM1pq + pM1pq' -pMpq1);
pM2pq = jacobian(M(:,2),[q1,q2,q3]);
pMpq2 = diff(M,q2);
C2 = 1/2*(pM2pq + pM2pq' -pMpq2);
pM3pq = jacobian(M(:,3),[q1,q2,q3]);
pMpq3 = diff(M,q3);
C3 = 1/2*(pM3pq + pM3pq' -pMpq3);
Cl1 = [qd1,qd2,qd3]*C1;
Cl2 = [qd1,qd2,qd3]*C2;
Cl3 = [qd1,qd2,qd3]*C3;

C = [Cl1;Cl2;Cl3];
% \frac{\partial L}{\partial q}
G1 = diff(P,q1);
G2 = diff(P,q2);
G3 = diff(P,q3);

G = [G1;G2;G3];

%%
Dq = subs(M,[ms,ls,g,Ic1s,Ic2s,Ic3s],[msn,lsn,gn,Ic1n,Ic2n,Ic3n]);
Cq = subs(C,[ms,ls,g,Ic1s,Ic2s,Ic3s],[msn,lsn,gn,Ic1n,Ic2n,Ic3n]);
Gq = subs(G,[ms,ls,g,Ic1s,Ic2s,Ic3s],[msn,lsn,gn,Ic1n,Ic2n,Ic3n]);
save('DCG','Dq','Cq','Gq')