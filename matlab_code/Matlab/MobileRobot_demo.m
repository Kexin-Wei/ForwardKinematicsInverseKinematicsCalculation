clc;clear;close all;
rob = MobileRobot();
%%
rob.dh_table

base_vec = [ pi/4, 1 , 1]; % theta0 x y
arm_vec  = [ pi/2, pi/4, pi/2]; %theta1 theta2 theta3
rob.set_dof(base_vec, arm_vec);

rob.l0
rob.theta1
rob.dh_table

rob.Ts{3}
rob = rob.cal_ts();
rob.Ts{3}

%% draw rob
close all
rob = rob.reset_dof()
rob = rob.draw();
%% base move
close all
rob = rob.reset_dof()
rob = rob.draw();

rob = rob.set_dof([0,100,0],[0,0,0]);
rob = rob.draw();

rob = rob.set_dof([pi/2,0,100],[0,0,0]);
rob = rob.draw();

%% arm move
close all
rob = rob.reset_dof()
rob.draw()

rob = rob.set_dof([0,0,0],[pi/4,-pi/4,pi/4])
rob = rob.draw();

%% all set
close all
rob = rob.reset_dof();
rob.draw();

rob = rob.set_dof([pi/4, 100, 200], [-pi/4, pi/4, -pi/4]);
rob = rob.draw();