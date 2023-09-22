classdef DualQuaternion
% q= qr+e*qd
% qr: real part 
%     a Hamilton type (q0,q1,q2,q3) quaternion class
% qd: dual part
%     a Hamilton type (q4,q5,q6,q7) quaternion class
% q: (q0,q1,q2,q3,q4,q5,q6,q7)
properties
        v (1,8) % all elements
    end
    methods
        
        %============================
        function r=DualQuaternion(q)
            % Initial the quaternion q is a (1,8) array /list
            r.v=q;
        end      
        %============================
        function rr=r(q)
           % extract the real part of q
           rr=Quaternion(q.v(1:4));
        end
        %============================
        function rd=d(q)
           % extract the dual part of q
           rd=Quaternion(q.v(5:8));
        end
        %============================
        function r=plus(q1,q2) 
            % + operator
            r=DualQuaternion(q1.v+q2.v);
        end
        %============================
        function r=minus(q1,q2)
            % - operator
            r=DualQuaternion(q1.v-q2.v);
        end  
        %============================
        function r=mtimes(q1,q2) 
            % * opetator: support q*q and scalar*q
            cq1=class(q1);
            cq2=class(q2);
            cq="DualQuaternion";
            cd="double";
            
            % a*b
            if cq1==cq && cq2==cq
                rr=q1.r*q2.r;
                rd=q1.r*q2.d+q1.d*q2.r;
                r=DualQuaternion([rr.v,rd.v]);
                return
            end
            
            % scalar times
            if cq1==cd && cq2==cq
                r=DualQuaternion(q1*q2.v);
                return
            end
            if cq1==cq && cq2==cd
                r=DualQuaternion(q2*q1.v);
                return
            end
            
        end
        
        %============================
        function r=conj(q)
            % Conjugate of the input quaternion
            rr=q.r.conj;
            rd=q.d.conj;
            r=DualQuaternion([rr.v,rd.v]);
        end
        
        %============================
        function r=swap(q)
            % swap the real and dual part
            r=DualQuaternion([q.d.v,q.r.v]);
        end
        
        %============================
        function r=cross(q1,q2)
            % cross product            
            rr=cross(q1.r,q2.r);
            rd=cross(q1.d,q2.r)+cross(q1.r,q2.d);
            
            r=DualQuaternion([rr.v,rd.v]);
        end
        
        %============================
        function r=vec(q)
           % take the vector of the q
           rr=q.r.v;
           rd=q.d.v;
           rr(1)=0;
           rd(1)=0;
           r=DualQuaternion([rr,rd]);
        end
        
        %============================
        function r=times(q1,q2)
            % dot product
            r=(q1.conj*q2+q2.conj*q1)*(1/2);
        end
        %============================
        function r=cdot(q1,q2)
           % dual quaternion circle product
           % ar.br+ad.bd         
           rr=q1.r.*q2.r+q1.d.*q2.d;
           rd=zeros(1,4);
           r=DualQuaternion([rr.v,rd]);           
        end
            
       
        %============================
        function r=normd(q)
            % Norm of the input quaternion
            r=q*q.conj;
        end
    
        %============================
        function r=norm(q)
           r=cdot(q,q); 
        end
        %============================
        function r=normalize(q)
            % Normalize the input quaternion
            rnormd=q.normd;
            r=DualQuaternion(q.v/sqrt(rnormd.v(1)));
        end
        %============================
        function rq=real(q)
            % return the real quaternion of dual quaternion
            rq=Quaternion(q.v(1:4));
        end
        %============================
        function rq=dual(q)
            % return the real quaternion of dual quaternion
            rq=Quaternion(q.v(5:8));
        end
    end
end