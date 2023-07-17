clc;clear; close all
rob = MobileRobot();
%% run simulink
%%
qval = q.Data;
t = q.Time;
t_idx=find(t>10);

%%
figure()
for i=1:t_idx(1)

rob = rob.set_dof([0,0,0],qval(i,:));
rob.draw();

end

title("Hand Shaking")