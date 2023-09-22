function rdq=DQFromQTvec(qYX,tYX_X)
   % build a dual quaternion from quaternion and translation vector
   % qYX: quaternion, or translation, ep. from reference frame D to B = qBD
   % tYX_X: translation vector, seen in the frame D = tBD_D
   t_quaternion = Quaternion([0,tYX_X]);
   rr = qYX;
   rd = (t_quaternion*qYX)*(1/2);
   rdq = DualQuaternion([rr.v, rd.v]);
end