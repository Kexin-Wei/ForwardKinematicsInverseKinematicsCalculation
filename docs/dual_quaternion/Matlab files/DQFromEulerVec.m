function rdq=DQFromEulerVec(theta,n,r)
   % build a dual quaternion from euler angle, rotate axism and
   % vector
   % theta: rotate angle, 
   % n: the rotate axis unit vector, 
   % r: translation vector
    q=QFromEuler(theta,n);
    rdq=DQFromQTvec(q,r);
end