% desired motion respect to molynia orbit 
clc;clear;close all
%% satellite initial
G=6.67428e-11;
M=5.9742e24;
mu=G*M;
E=0.74;
A=2.66e7; %10*7m

TA=180; % start at perigee
[rTI,vTI,wTI,qTI]=Target(TA);

% time set
dt=1;
T=2*pi*A^(3/2)/sqrt(mu)*1.5;
t_set=0:dt:T;
n_t=size(t_set,2);

rd=zeros(3,n_t);
rt=zeros(3,n_t);
vc=zeros(3,n_t);
wc=zeros(3,n_t);

%% %% Motion Simulation
for i=1:n_t
    t=t_set(i);
    % ======= T to I ======
    % get  wTI_I_q, rTI_I_q, qTI
    % get wTI_I_dq,    dqTI,
    aTI=molniya_a(rTI,mu);
    TI=SimpleReferFrame(rTI,vTI,wTI,qTI);
    
    % ======= D to T =======
    % get wDT_T_q, rDT_T_q, qDT
    % get wDT_T_dq,   dqDT,
    DT=Desired(t);
    
    % qDI= qTI * qDT
    % dqDI= dqTI * dqDT
    DI_q=TI.q*DT.q;
    DI_dq=TI.dq*DT.dq;
    
    % wDT_D_dq= dqDT.conj * wDT_T *
    % wTI_D_dq= dqDI.conj * wTI_I *
    % wDI_D_dq= wDT_D + wTI_D
    wDT_D_dq= DT.dq.conj * DT.wdq * DT.dq;
    wTI_D_dq= DI_dq.conj * TI.wdq * DI_dq;
    wDI_D_dq= wDT_D_dq+wTI_D_dq;
    
    % rDT_D_q= qDT.conj * rDT_T_q *
    % rTI_D_q= qDI.conj * rTI_I_q *
    % rDI_D_q= rDT_D + rTI_D
    rDT_D_q= DT.q.conj * DT.r.q * DT.q;
    rTI_D_q= DI_q.conj * TI.r.q * DI_q;
    rDI_D_q= rDT_D_q + rTI_D_q;
    
    % wDI_D_q= wDI_D_dq.rel
    wDI_D_q= wDI_D_dq.real;
    
    % vDI_D_q= wDI_D_dq.dual - rDI_D_q * wDI_D_q
    vDI_D_q= wDI_D_dq.dual - rDI_D_q * wDI_D_q;
   
    % collect date of v w
    vc(:,i)=vDI_D_q.v(2:4);
    wc(:,i)=wDI_D_q.v(2:4);
    
    % collect trajecotry
    rDI_I_q=DI_q * rDI_D_q * DI_q.conj;
    rDI_I=rDI_I_q.v(2:4);
    rd(:,i)=rDI_I;
    rt(:,i)=TI.r.v;
    
    % Target Update from a
    rTI=TI.r.v+TI.v.v*dt;
    vTI=TI.v.v+aTI*dt;
    wTI= cross(rTI,vTI)/(norm(rTI)*norm(rTI));
    
end
%% reshow the fig3
figure()
subplot(3,2,1)
plot(t_set,wc(1,:))
grid on
xlabel('Time (s)')
ylabel('p^{D}_{D/I}(m/s)')

subplot(3,2,2)
plot(t_set,vc(1,:))
grid on
xlabel('Time (s)')
ylabel('u^{D}_{D/I}(m/s)')

subplot(3,2,3)
plot(t_set,wc(2,:))
grid on
xlabel('Time (s)')
ylabel('q^{D}_{D/I}(m/s)')

subplot(3,2,4)
plot(t_set,vc(2,:))
grid on
xlabel('Time (s)')
ylabel('v^{D}_{D/I}(m/s)')

subplot(3,2,5)
plot(t_set,wc(3,:))
grid on
xlabel('Time (s)')
ylabel('r^{D}_{D/I}(m/s)')

subplot(3,2,6)
plot(t_set,vc(3,:))
grid on
xlabel('Time (s)')
ylabel('w^{D}_{D/I}(m/s)')
%% trajectory
figure()
plot3(rt(1,:),rt(2,:),rt(3,:),'b')
hold on
grid on
plot3(rd(1,:),rd(2,:),rd(3,:),'r--')
legend('Target','Desired')
view([-60,20])
axis equal
%% compare of original
figure()
load('molynia_orbit_radius.mat')
rc=rc';
plot3(rc(1,:),rc(2,:),rc(3,:),'k')
hold on
grid on
plot3(rt(1,:),rt(2,:),rt(3,:),'r-.')
legend('Target','Original of Target')
view([-60,20])
axis equal
%% inside function
function aTI=molniya_a(rTI,mu)
    aTI=-mu*rTI/(norm(rTI)^3);
end
