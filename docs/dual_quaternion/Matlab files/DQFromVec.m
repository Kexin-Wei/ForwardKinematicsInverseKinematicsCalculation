function dq=DQFromVec(v)
    % v: (1,3) vector
    % dq: v build a pure quaternion for real part, 0 for dual part
    dq=DualQuaternion([0,v,zeros(1,4)]);
end