classdef VQ
    % a container for vector, quaternion
    properties
        v (1,3)
        q
    end
    
    methods
        function class=VQ(v)
            % input: v is a vector
            class.v=v;
            class.q=Quaternion([0,v]);
            
        end
        
        function rcY=FrameS(cX,qYX)
            % c.q is from YX_X to YX_Y use q(quaternion)
            rcY=cX;
            rcY.q=qYX.conj*cX.q*qYX;
            rcY.v=rcY.q.v(2:4);
        end
    end
    
    
end