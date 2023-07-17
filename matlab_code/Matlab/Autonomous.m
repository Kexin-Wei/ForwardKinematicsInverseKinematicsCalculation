% Parameters
m1 = 0;
m2 = 0;
m3 = 0;

w1 = 0;
w2 = 0;
w3 = 0;

l1 = 0;
l2 = 0;
l3 = 0;

h1 = 0;
h2 = 0;
h3 = 0;

theta1_deg = 0;
theta2_deg = 0;
theta3_deg = 0;

theta1 = theta1_deg*pi/180;
theta2 = theta2_deg*pi/180;
theta3 = theta3_deg*pi/180;

g = 9.8; % gravity constant

%% Lagrangian-euler dynamic equation in joint space from Q1
Dq = zeros(3,3); % inertia matrix
Cq = zeros(3,3); % Coriolis and centrifugal matrix
Gq = zeros(3,1); % Gravity matrix


% inertia tensor

Ic1 = zeros(3,3);
Ic2 = zeros(3,3);
Ic3 = zeros(3,3);

Ic1(1,1) = m1*(w1^2+l1^2)/12;
Ic1(2,2) = m1*w1^2/6;
Ic1(3,3) = m1*(w1^2_l1^2)/12;

Ic2(1,1) = m2*w2^2/6;
Ic2(2,2) = m2*(w^2^2+l2^2)/12;
Ic2(3,3) = m2*(w2^2+l2^2)/12;

Ic3(1,1) = m3*w3^2/6;
Ic3(2,2) = m3*(w3^2+l3^2)/12;
Ic3(3,3) = m3*(w3^2+l3^2)/12;


% Initialization of Dq
Dq(1,1) = Ic1(2,2)+(m2*l2^2*cos(theta2)^2)+Ic2(1,1)*sin(theta2)^2+...
    Ic2(2,2)*cos(theta2)^2+m3*l2^2+cos(theta2)^2+(m3*l3^2*cos(theta2+theta3)^2)/4 +...
    m3*l2*l3*cos(theta2)*cos(theta2+theta3)+Ic3(1,1)(sin(theta2+theta3)^2+Ic3(2,2)*cos(theta2+theta3)^2;
Dq(1,2) = 0;
Dq(1,3) = 0;

Dq(2,1) = Dq(1,2);
Dq(2,2) = (m^2*l2^2/4)+Ic2(3,3)+m3*l2^2+)(m3*l3^2/4)+m3*l2*l3*cos(theta3)+Ic3(3,3);
Dq(2,3) = (m3*l3^2/4)+0.5*m3*l2*l3*cos(theta3)+Ic3(3,3);

Dq(3,1) = Dq(1,3);
Dq(3,2) = Dq(2,3);
Dq(3,3) = (m3*l3^2/4)+Ic3(3,3);

% Initialization of Cq

h1 = 0;
h2 = 0;
h3 = 0;

C(1,1) = (h1/2)*q2dot+(h2/2)q3dot;
C(1,2) = (h1/2)*q1dot;
C(1,3) = (h2/2)q1dot;

C(2,1) = -(h1/2)*q1dot;
C(2,2) = (h3/2)*q3dot;
C(2,3) = (h3/2)*q2dot + (h3/2)*q3dot;

C(3,1) = -(h2/2)*q1dot;
C(3,2) = -(h3/2)*q2dot;
C(3,3) = 0;

% Initialization of Gq

G(1,1) = 0;
G(2,1) = -(m2*g*l2*cos(theta2)/2)-(m3*g*l2*cos(theta2)-(m3*g*l3*cos(theta2+theta3)/2);
G(3,1) = -m3*g*l3*cos(theta2+theta3)/2;


%% Lagrangian-euler dynamic equation in task space (conversion)

J = zeros(3,3); % Jacobian matrix

J(1,1) = -l2*sin(theta1)*cos(theta2)-l3*sin(theta1)*cos(theta2+theta3);
J(1,2) = -l2*cos(theta1)*sin(theta2)-l3*cos(theta1)*sin(theta2+theta3);
J(1,3) = -l3*cos(theta1)*sin(theta2+theta3);

J(2,1) = l2*cos(theta1)*cos(theta2)+l3*cos(theta1)*cos(theta2+theta3);
J(2,2) = -l2*sin(theta1)*sin(theta2)-l3*sin(theta1)*cos(theta2+theta3);
J(2,3) = -l3*sin(theta1)*sin(theta2+theta3);

J(3,1) = 0;
J(3,2) = -l2*cos(theta2)-l3*cos(theta2+theta3);
J(3,3) = -3*cos(theta2+theta3);

Dx = (inv(J)')*Dq*inv(J);
Cx = (inv(J)')*(Cq-Dq*inv(J)Jdot)*inv(J);
Gx = (inv(J)')*Gq;

%% Impedance control model

M = zeros(3,3);
B = zeros(3,3);
K = zeros(3,3);

F_env = zeros(3,3);

x = zeros(3,1);
x_d = zeros(3,1);

xdot = diff(x);
x_ddot = diff(x_d);

F_prime = inv(M)*(B*(xdot-x_ddot)+K(x-x_dot)+F_env);
alpha = Dx;
beta = Cx*xdot+Gx+F_env;

F = alpha*F_prime+beta;



