classdef SimpleReferFrame
    % define a Reference Frame Y respect to frame X
    % Input:
    % r: rYX_Y
    % r,w,v,a,aa all seen in X
    % q: quaternion, a rotation form X to Y
    %
    % Define:
    % r: a class has vec(r.v) and quaternion(r.q)
    % w,v,a,aa: same like r
    % q: quaternion, a rotation form X to Y
    % dq: X displaces and rotates to Y
    % wdq: dual quaternion of wYX_Y
    % rdq: dual quaternion of rYX_Y
    
    properties
        r % displacement
        w % angular velocity
        v % linear velocity

        q  % quaternion
        dq % dual quaternion
        wdq % dual quaternion of w
        rdq % dual quaternion of r
    end
    
    methods
        %====================
        function c=SimpleReferFrame(r,v,w,q)
            % define function
            c.r=VQ(r);
            c.v=VQ(v);
            c.w=VQ(w);       
            
            c.q=q;
            c.dq=DQFromQTvec(q,r);
            
            c.rdq=DualQuaternion([0,r,zeros(1,4)]);
            wdq_d=c.v.q+cross(c.r.q,c.w.q);
            c.wdq=DualQuaternion([c.w.q.v,wdq_d.v]);
        end
        
        %===================
        function rcYX_Y=FrameSwap(cYX_X)
            % Swap a reference frame elements from rYX_X to rYX_Y
            rcYX_Y=cYX_X;
            rcYX_Y.r=FrameS(cYX_X.r,cYX_X.q);
            rcYX_Y.w=FrameS(cYX_X.r,cYX_X.q);
            rcYX_Y.v=FrameS(cYX_X.r,cYX_X.q);
            
            rcYX_Y.wdq=cYX_X.dq.conj*cYX_X.wdq*cYX_X.dq;
            rcYX_Y.rdq=cYX_X.dq.conj*cYX_X.rdq*cYX_X.dq;
        end
        
    end
    
end