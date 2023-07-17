rob = MobileRobot();
qval = q.Data;
%%
figure()
for i=1:size(qval,1)

rob = rob.set_dof([0,0,0],qval(i,:));
rob.draw();

end