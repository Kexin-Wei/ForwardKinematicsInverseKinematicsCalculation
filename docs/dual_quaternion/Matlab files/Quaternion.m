classdef Quaternion
    % a Hamilton type (q0,q1,q2,q3) quaternion class
    properties
        v (1,4) % value of the quaternion
    end
    methods
        %============================
        function r=Quaternion(q)
            % Initial the quaternion
            r.v=q;
        end
        
        %============================
        function r=plus(q1,q2) 
            % + operator
            r=Quaternion(q1.v+q2.v);
        end    
        
        %============================
        function r=minus(q1,q2)
            % - operator
            r=Quaternion(q1.v-q2.v);
        end
        
        %============================
        function r=mtimes(q1,q2) 
            % * opetator: support q*q and scalar*q
            cq1=class(q1);
            cq2=class(q2);
            cq="Quaternion";
            cd="double";
            
            % a*b
            if cq1==cq && cq2==cq
                q10=q1.v(1);q1vec=q1.v(2:4);
                q20=q2.v(1);q2vec=q2.v(2:4);
                r0=q10*q20-q1vec*q2vec';
                rvec=q10*q2vec+q20*q1vec+cross(q1vec,q2vec);
                r=Quaternion([r0,rvec]);
                return
            end
            
            % scalar times
            if cq1==cd && cq2==cq
                r=Quaternion(q1*q2.v);
                return
            end
            if cq1==cq && cq2==cd
                r=Quaternion(q2*q1.v);
                return
            end
            
        end
        
        %============================
        function r=conj(q)
            % Conjugate of the input quaternion
            q0=q.v(1);qvec=q.v(2:4);
            r=Quaternion([q0,-qvec]);
        end
        
        %============================
        function r=norm(q)
             % Norm of the input quaternion
            qc=q.conj;
            r=q*qc;
        end
        
        %============================
        function r=normalize(q)
            % Normalize the input quaternion
            length=norm(q);
            r=Quaternion(q.v/sqrt(length.v(1)));
        end
        
        %============================
        function r=cross(q1,q2)
            % cross product
            q10=q1.v(1);q1vec=q1.v(2:4);
            q20=q2.v(1);q2vec=q2.v(2:4);
            
            rvec=q10*q2vec+q20*q1vec+cross(q1vec,q2vec);
            r=Quaternion([0,rvec]);
        end
        
        %============================
        function r=vec(q)
           % take the vector part of q
           rv=q.v;
           rv(1)=0;
           r=Quaternion(rv);
        end
        
        %============================
        function r=times(q1,q2)
            % dot product
            r=(q1.conj*q2+q2.conj*q1)*(1/2);
        end
      
        %============================
        function rBA_Y=Qrotate(qYX,rBA_X)
            % rBA_Y: rotate rBA_X use qYX
            rBA_Y=qYX.conj*rBA_X*qYX;
        end
    end
        
end