clc;clear;close all
syms q1 q2 q3 real
syms qd1 qd2 qd3 real
syms qdd1 qdd2 qdd3 real
q_vec   = [q1,q2,q3]';
qd_vec  = [qd1,qd2,qd3]';
qdd_vec = [qdd1,qdd2,qdd3]';
load DCG
%% init condition
qt_vec   = [0, 0, 0]';
qdt_vec  = [0, 0, 0]';

dt = 0.01;
time = 4;
ite = 0:dt:time;

taus = zeros(3,length(ite));
qs   = zeros(3,length(ite));
%%
for i = 1:length(ite)
    Dqt = double(subs(Dq,q_vec,qt_vec));
    Cqt = double(subs(Cq,[q_vec,qd_vec],[qt_vec,qdt_vec]));
    Gqt = double(subs(Gq,q_vec,qt_vec));
    
    qddt_vec = Dqt\(taus(:,i)- Cqt*qdt_vec - Gqt);
    qdt_vec = qdt_vec + qddt_vec * dt;
    qt_vec = qt_vec + qdt_vec * dt;
    qs(:,i) = qt_vec;
end

%%
close all
rob = MobileRobot();
draw_idx = round(quantile(1:1:length(ite),0:0.1:1));
xyzs = zeros(3,length(ite));
for i=1:length(ite)
    xyzs(:,i) = rob.forward([0,0,0],qs(:,i));
end
patch(xyzs(1,:),xyzs(2,:),xyzs(3,:),ite,'EdgeColor','interp','FaceColor','none')
hold on
plot3(xyzs(1,1),xyzs(2,1),xyzs(3,1),'r*')
plot3(xyzs(1,end),xyzs(2,end),xyzs(3,end),'ro')
for i=1:length(draw_idx)
    rob.draw([0,0,0],qs(:,draw_idx(i)));
end
view(45,45)            
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title("Hand Shaking")