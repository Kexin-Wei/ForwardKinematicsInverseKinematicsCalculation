% ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜
function [r, v, q] = sv_from_coe(coe,mu)

% ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜
% This function computes the state vector (r,v) from the
% classical orbital elements (coe).
%
% mu - gravitational parameter (kmˆ3; sˆ2)
% coe - orbital elements [h e RA incl w TA]
% where
% h = angular momentum (kmˆ2/s)
% e = eccentricity
% RA = right ascension of the ascending node (rad)
% incl = inclination of the orbit (rad)
% w = argument of perigee (rad)
% TA = true anomaly (rad)
% R3_w - Rotation matrix about the z-axis through the angle w
% R1_i - Rotation matrix about the x-axis through the angle i
% R3_W - Rotation matrix about the z-axis through the angle RA
% Q_pX - Matrix of the transformation from perifocal to
% geocentric equatorial frame
% rp - position vector in the perifocal frame (km)
% vp - velocity vector in the perifocal frame (km/s)
% r - position vector in the geocentric equatorial frame
% (km)
% v - velocity vector in the geocentric equatorial frame
% (km/s)
%
% User M-functions required: none
% ------------------------------------------------------------

% global mu

h = coe(1);
e = coe(2);
RA = coe(3);
incl = coe(4);
w = coe(5);
TA = coe(6);

%...Equations 4.37 and 4.38 (rp and vp are column vectors):
rp = (h^2/mu) * (1/(1 + e*cos(TA))) * (cos(TA)*[1;0;0]+ sin(TA)*[0;1;0]);
vp = (mu/h) * (-sin(TA)*[1;0;0] + (e + cos(TA))*[0;1;0]);

%=== quaternion
q1=QFromEuler(RA,[0,0,1]);
q2=QFromEuler(incl,[1,0,0]);
q3=QFromEuler(w,[0,0,1]);
q=q1*q2*q3;

% r v quaternion
rq=QFromVec(rp');
vq=QFromVec(vp');

rq2=q*rq*q.conj;
vq2=q*vq*q.conj;

r=rq2.v(2:4);
v=vq2.v(2:4);



% ˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜˜