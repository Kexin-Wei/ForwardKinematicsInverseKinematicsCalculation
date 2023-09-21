function plotvec3(vector,start_point,color)
new_matrix=[start_point;vector+start_point]'; %3x2
x=new_matrix(1,:);
y=new_matrix(2,:);
z=new_matrix(3,:);
plot3(x,y,z,color);
hold on 

scolor=append(color(1),'*');
scatter3(x(2),y(2),z(2),scolor)
xlabel('x');
ylabel('y');
zlabel('z');
grid on
end