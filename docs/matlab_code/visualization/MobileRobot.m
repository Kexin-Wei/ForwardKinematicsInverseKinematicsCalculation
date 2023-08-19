classdef MobileRobot
    properties
        % default link length (Unit:centimeter)
        l1       (1,1) double {mustBePositive,mustBeNumeric} = 40      
        l2       (1,1) double {mustBePositive,mustBeNumeric} = 30
        l3       (1,1) double {mustBePositive,mustBeNumeric} = 20
        
        %dof
        theta0   (1,1) double = 0
        theta1   (1,1) double = 0
        theta2   (1,1) double = 0
        theta3   (1,1) double = 0
        bx       (1,1) double = 0
        by       (1,1) double = 0
        
        % dh
        Ts       (1,4) cell        
        dh_table = [0, 40, -pi/2,  0;
                    0,  0,     0, 30;
                    0,  0,     0, 20];
        
    end
    
    methods
        % ==========================
        function rob = update_dh(rob)            
            rob.dh_table(1,1) = rob.theta1;
            rob.dh_table(1,2) = rob.l1;
            rob.dh_table(2,1) = rob.theta2;
            rob.dh_table(2,4) = rob.l2;
            rob.dh_table(3,1) = rob.theta3;
            rob.dh_table(3,4) = rob.l3;      
        end
        % ==========================
        function rob = reset_link_length(rob,l1,l2,l3)
            rob.l1       = l1;
            rob.l2       = l2;
            rob.l3       = l3;
            rob = rob.update_dh();
        end
        % ==========================
        function rob = set_dof(rob,first_vec,second_vec)
            % first_vec:  theta_0,      bx,      by 
            % second_vec: theta_1, theta_2, theta_3
            rob.theta0 = first_vec(1);
            rob.bx      = first_vec(2);
            rob.by      = first_vec(3);
            rob.theta1 = second_vec(1);
            rob.theta2 = second_vec(2);
            rob.theta3 = second_vec(3);
            rob = rob.update_dh();
        end
        % ==========================
        function rob = reset_dof(rob)
            rob = rob.set_dof([0,0,0],[0,0,0]);
        end
        % ==========================
        function rob = cal_ts(rob)
            T   = eye(4);
            Tb0 = [ cos(rob.theta0), -sin(rob.theta0), 0, rob.bx;
                    sin(rob.theta0),  cos(rob.theta0), 0, rob.by;
                                  0,                0, 1,     5;
                                  0,                0, 0,     1];
            T = T*Tb0;    
            rob.Ts{1} = T;
            for i =1:size(rob.dh_table,1)
                T=T*rob.dh_matrix(rob.dh_table(i,:));
                rob.Ts{i+1}=T;
            end
        end
        % ==========================
        function rob = draw(rob)
            rob = rob.cal_ts();
            bs_clr = [0.25,0.25,0.25];
            jt_clr  = [0.5,0.5,0.5];
            % ===== base / Tb0
            h_bs  = 14; 
            db    = 2;
            w_bss = 22;
            % b1
            [x,y,z] = rob.obj(1,h_bs,'cb'); 
            x = x*58 -14;
            y = y*74;
            z = z - db;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            rob.draw_obj(x,y,z,bs_clr);  
            % b2
            [x,y,z] = rob.obj(1,h_bs,'cb'); 
            x = x*22 - 54;           
            y = y*30;
            z = z - db;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            rob.draw_obj(x,y,z,bs_clr);
            % b3
            [xt,yt,zt] = rob.obj(h_bs,w_bss,'tp');             
            x = -zt-43;
            y = -yt/h_bs*w_bss-15;
            z = xt;
            z = z - db + h_bs/2;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            rob.draw_obj(x,y,z,bs_clr);
            % b4
            [xt,yt,zt] = rob.obj(h_bs,w_bss,'tp');             
            x = -zt-43;
            y = yt/h_bs*w_bss+15;
            z = xt;
            z = z - db + h_bs/2;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            rob.draw_obj(x,y,z,bs_clr);
            % ===== wheel / Tb0
            w_w = 5;
            w_r = 20;
            % w1
            [xt,yt,zt] = rob.obj(w_r,w_w,'cy');             
            x = xt;
            y = -zt - 37.5;
            z = yt;
            z = z + 5;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            [~,color]=rob.draw_obj(x,y,z,[0.9,0.9,0.9]);  
            % w2
            [xt,yt,zt] = rob.obj(w_r,w_w,'cy');            
            x = xt;
            y = zt + 37.5;
            z = yt;
            z = z + 5;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            rob.draw_obj(x,y,z,color);
            % w3
            [xt,yt,zt] = rob.obj(15,w_w,'cy');             
            x = xt - 45;
            y = -zt + w_w/2;
            z = yt;
            z = z + 2.5;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{1});
            rob.draw_obj(x,y,z,color); 
            % ===== link1 / Tb0
            w1  = 14; w2  = 12; w3 =10;
            [xt,yt,zt] = rob.obj(w1,rob.l1,'cb');
            x = xt;
            y = -zt+rob.l1;
            z = yt;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{2});
            rob.draw_obj(x,y,z,[1,0.39,0.28]);
            [x,y,z]=rob.obj(w1*1.1,w1*1.1,'cy');
            z = z - w1*1.1/2;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{2});
            rob.draw_obj(x,y,z,jt_clr);
            % ====== link2 / Tb1
            [xt,yt,zt] = rob.obj(w2,rob.l2,'cb');
            x = zt-rob.l2;
            y = xt;
            z = yt;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{3});
            rob.draw_obj(x,y,z,[1,0.63,0.72]);
            [x,y,z]=rob.obj(w2*1.1,w2*1.1,'cy');
            z = z - w2*1.1/2;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{3});
            rob.draw_obj(x,y,z,jt_clr);
            % ====== link3 / Tb2
            lw3 = rob.l3*0.7;
            [xt,yt,zt] = rob.obj(w3,lw3,'cb');
            x = zt-rob.l3;
            y = xt;
            z = yt;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{4});
            [~,color] = rob.draw_obj(x,y,z,[0.6,0.8,0.2]);
            % gripper
            gw = w3*1.1;            
            gh = rob.l3*0.3;
            [xt,yt,zt]= rob.obj(gw,gh*.5,'cb');
            x = zt-gh*1.5;
            y = xt;
            z = yt*3;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{4});
            rob.draw_obj(x,y,z,color);            
            [xt,yt,zt]= rob.obj(gw,gh,'tp');
            x = zt-gh;
            y = xt;
            z = .5*yt+gw;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{4});
            cl=rob.draw_obj(x,y,z,color);
            [xt,yt,zt]= rob.obj(gw,gh,'tp');
            x = zt-gh;
            y = xt;
            z = -.5*yt-gw;
            [x,y,z]= rob.trans_ref(x,y,z,rob.Ts{4});
            cr=rob.draw_obj(x,y,z,color);
            % xyz 6
            x = (cl(2,1)+cr(2,1))/2;
            y = (cl(2,2)+cr(2,2))/2;
            z = (cl(2,3)+cr(2,3))/2;
            plot3(x,y,z,'r*')                        
            view(45,45)
            title("Mobile Robot")
            axis equal
            xlabel('x')
            ylabel('y')
            zlabel('z')
        end
    end
    % ========= Static Functions ============================= 
    methods (Static)
        function A = dh_matrix(in_vector)
             assert(isequal(size(in_vector),[1,4]) || isequal(size(in_vector),[4,1]), ...
             "Please check the input, should be a 4 dimensional vector in column or row")
             theta = in_vector(1);             
             d = in_vector(2);
             alpha = in_vector(3);
             a = in_vector(4);
                
             A =[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
                 sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                          0,             sin(alpha),             cos(alpha),            d;
                          0,                      0,                      0,            1];
        end
        
        function [x,y,z] = obj(w,l,type)
            switch type
                case "cy"   %cylinder
                    [x,y,z] = cylinder(w/2); %diameter
                    z = z*l;
                case "cb"   %cuboid
                    xr = linspace(-w/2,w/2,21);
                    yr = linspace(-w/2,w/2,21);
                    wc = w/2*ones(1,21);
                    xl = [ xr, wc, -xr, -wc];
                    yl = [-wc, yr,  wc, -yr];
                    x  = [xl;xl];
                    y  = [yl;yl];
                    z  = [zeros(1,size(x,2)); l*ones(1,size(x,2))]; 
                case "tp"   %triangular prism
                    xr=linspace(-w/2,w/2,21);
                    yr=linspace(0,w,21);
                    wc=w*ones(1,21);
                    x1=[xr,wc/2,-xr,-wc/2];
                    y1=[0*wc,yr,wc,yr(end:-1:1)];
                    z1=zeros(1,size(x1,2));
                    x2=x1;
                    y2=0*y1;
                    z2=l*ones(1,size(x2,2));
                    x=[x1;x2];
                    y=[y1;y2];
                    z=[z1;z2];
                case "sp" %sphere
                    w = max(w/2,l/2); %diameter
                    [x,y,z]=sphere;
                    x = x*w;
                    y = y*w;
                    z = z*w;
                otherwise
                    x = zeros(2,21);
                    y = zeros(2,21);
                    z = zeros(2,21);
            end
        end
        
        function varargout = draw_obj(x,y,z,color)  
            % varargout:https://www.mathworks.com/help/matlab/matlab_prog/support-variable-number-of-outputs.html
            if nargin == 3
                color = rand(1,3);
            end
            mesh(x,y,z,'FaceColor',color,'LineStyle','none')
            hold on
            fill3(x(1,:),y(1,:),z(1,:),color)
            fill3(x(2,:),y(2,:),z(2,:),color)
            
            c_x1 = (max(x(1,:))+min(x(1,:)))/2;
            c_y1 = (max(y(1,:))+min(y(1,:)))/2;
            c_z1 = (max(z(1,:))+min(z(1,:)))/2;
            c_x2 = (max(x(2,:))+min(x(2,:)))/2;
            c_y2 = (max(y(2,:))+min(y(2,:)))/2;
            c_z2 = (max(z(2,:))+min(z(2,:)))/2;
            nOutputs = nargout;
            varargout = cell(1,nOutputs);
           
            center = [c_x1,c_y1,c_z1;
                      c_x2,c_y2,c_z2];
                  
             if nOutputs > 0
                 varargout{1} = center;
             end
             if nOutputs > 1
                 varargout{2} = color;
             end
        end
        
        function [x,y,z] = trans_ref(x,y,z,T)
           len = size(x,2);
           p1 = [x(1,:);y(1,:);z(1,:);ones(1,len)];
           p2 = [x(2,:);y(2,:);z(2,:);ones(1,len)];
           p11 = T*p1;
           p22 = T*p2;
           x = [p11(1,:);p22(1,:)];
           y = [p11(2,:);p22(2,:)];
           z = [p11(3,:);p22(3,:)];
        end
    end
end