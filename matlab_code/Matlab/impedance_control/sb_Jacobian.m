clc;clear;close all
syms q1 q2 q3 real
syms qd1 qd2 qd3 real
syms qdd1 qdd2 qdd3 real
q_vec   = [q1,q2,q3]';
qd_vec  = [qd1,qd2,qd3]';
qdd_vec = [qdd1,qdd2,qdd3]';

%% DH parameters
syms l1 l2 l3 real
ls  = [l1, l2, l3];
lsn = [0.4  0.3  0.2]; % link length(m) l1 l2 l3
dh  = [q1, l1, -pi/2,   0;
       q2,  0,     0,  l2;
       q3,  0,     0, l3];

n_dof = size(dh,1);
T  = eye(4);
for i=1:n_dof
    A = dh_matrix(dh(i,:));
    T = T*A;
end
r03 = T(1:3,4);

%%
J  = simplify(jacobian(r03,q_vec'));
Jd = simplify(diff(J,q1)*qd1 + diff(J,q2)*qd2 + diff(J,q3)*qd3);

Ja  = subs(J,ls,lsn);
Jad = subs(Jd,ls,lsn);
re  = subs(r03,ls,lsn);
save('Ja','Ja','Jad','re')