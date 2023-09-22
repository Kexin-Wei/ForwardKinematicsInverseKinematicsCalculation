function dq=DQFromRealQ(q)
    % q: rTI_I the quaternion of vector rTI in I
    % build up a dual quaternion has zero dual part 
    dq=DualQuaternion([q.v,zeros(1,4)]);

end