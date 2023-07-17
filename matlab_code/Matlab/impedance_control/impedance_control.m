clc;clear;close all
syms q1 q2 q3 real
syms qd1 qd2 qd3 real
syms qdd1 qdd2 qdd3 real
q_vec   = [q1,q2,q3]';
qd_vec  = [qd1,qd2,qd3]';
qdd_vec = [qdd1,qdd2,qdd3]';
load DCG
load Ja

%% Design 
dt   = 0.01;
time = 4;
ite  = 0:dt:time;

taus = zeros(3,length(ite));
Mm = [0.19,  0.04,  0.02;
      0.03,  0.035, 0.01;
      0.02,  0.01,  0.4];
Dm = [47,  22, 25;
      22,  56, 9;
      25,  9,  86];
Km = [320, 12, 26;
      12,  56, 9;
      25,  9,  86];

% init condition
qt_vec   = [0, asin(0.1/0.3), -asin(0.1/0.3)-asin(0.1/0.2)]';
qdt_vec  = [0, 0, 0]';

qs = zeros(3,length(ite));

% external force
Fe    = zeros(3,length(ite));
f_idx = find(ite>0.5 & ite<3.5);
f_A   = 20;
Fe(3,f_idx) = f_A*sin(2*ite(f_idx));
% figure()
% plot(ite,Fe)
% legend('F_x','F_y','F_z')
% title("F_{ext}")

% trajectory
xddotdot = zeros(3,length(ite));
xddot    = zeros(3,length(ite));
xinit    = double(subs(re,q_vec,qt_vec));
xd       = ones(3,length(ite)).*xinit;

% joint velocity limits
qdt_lim = 0.2*pi*ones(1,3)';
%%
for i = 1:length(ite)
    Mqt  = double(subs(Dq,q_vec,qt_vec));
    Cqt  = double(subs(Cq,[q_vec,qd_vec],[qt_vec,qdt_vec]));
    Gqt  = double(subs(Gq,q_vec,qt_vec));
    Jat  = double(subs(Ja,q_vec,qt_vec));
    Jadt = double(subs(Jad,[q_vec,qd_vec],[qt_vec,qdt_vec]));
    Jati = inv(Jat);
    
    Mxt = Jati' * Mqt / Jat;
    Cxt = Jati' * Cqt * Jati - Mxt * Jadt / Jat;
    Gxt = Jati' * Gqt;
    
    xt_vec = double(subs(re,q_vec,qt_vec));
    xdott_vec = Jat * qdt_vec;
    
    % choose one control model
    % model 1: Mm = Mxt
    Mm = Mxt;
    % 1.1 only for fixed point
    taus = Gqt + Jat'*(Km * (xd(:,i) - xt_vec) - Dm * xdott_vec); 
    % 1.2 follow traject
%     taus = Mqt/Jat*(xddotdot(:,i)-Jadt*qdt_vec) + Cqt*qdt_vec + Gqt +...
%            Jat'*(Dm*(xddot(:,i)-xdott_vec) + Km*(xd(:,i)-xt_vec));
%     % model 2: Mm manual, need shaping
%     taus = Mqt/Jat*(xddotdot(:,i)-Jadt*qdt_vec + ...
%            Mm\(Dm*(xddot(:,i)-xdott_vec) + Km*(xd(:,i)-xt_vec)))...
%            + Cqt*qdt_vec + Gqt +...
%            Jat'*(Mxt/Mm - eye(3))*Fe(:,i);
       
    qddt_vec = Mqt\(taus + Jat'*Fe(:,i) - Cqt*qdt_vec - Gqt);
    qdt_vec = qdt_vec + qddt_vec * dt;
    % joint speed limits
%     qdt_vec(qdt_vec > qdt_lim)   = qdt_lim(qdt_vec > qdt_lim);
%     qdt_vec(qdt_vec < - qdt_lim) = qdt_lim(qdt_vec < - qdt_lim);
    qt_vec = qt_vec + qdt_vec * dt;
    qs(:,i) = qt_vec;
end

%%
close all
rob = MobileRobot();
xyzs = zeros(3,length(ite));
for i=1:length(ite)
    xyzs(:,i) = rob.forward([0,0,0],qs(:,i));
end
patch(xyzs(1,:),xyzs(2,:),xyzs(3,:),ite,'EdgeColor','interp','FaceColor','none')
hold on
grid on
plot3(xyzs(1,1),xyzs(2,1),xyzs(3,1),'r*')
plot3(xyzs(1,end),xyzs(2,end),xyzs(3,end),'ro')
view(45,45)            
axis equal
xlabel('x')
ylabel('y')
zlabel('z')


%%
draw_idx = round(quantile(1:1:length(ite),0:0.2:1));
for i=1:length(draw_idx)
    rob.draw([0,0,0],qs(:,draw_idx(i)));
end
view(45,45)            
axis equal
xlabel('x')
ylabel('y')
zlabel('z')
title("Hand Shaking")