function J = Jacobian_line(Ts,pc,i,type)
    R = Ts{i}(1:3,1:3);
    zi_1 = R(:,3);

    switch type
        case 'p' % prismatic joint
            Jv = zi_1;
            Jw = zeros(3,1);

        case 'r' %revolute joint
            pi_1 = Ts{i}(1:3,4);
            Jv   = cross(zi_1,(pc-pi_1));
            Jw   = zi_1;

        otherwise
            disp("Choose 'r' or 'p' for joint type")
            Jv = [0,0,0];
            Jw = [0,0,0];
    end
    J = [Jv;Jw];
end