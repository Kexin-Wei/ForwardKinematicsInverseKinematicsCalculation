function Jm = Jacobian_matrix(Ts,i_dof,rp_table)
    % Ts:       cell type homegenous matrix set, n=n_dof+1
    % i_dof:    the i th dof
    % rp_table: a vector like ['r', 'p', 'r'] represents the joint type
    %           revolute or primatic
    Jm = [];
    % assume mass center at link middel
    pc = (Ts{i_dof}(1:3,4)+Ts{i_dof+1}(1:3,4))/2;
    
    for i = 1:length(rp_table)
        J = Jacobian_line(Ts,pc,i,rp_table(i));
        if i<=i_dof
            Jm = [Jm,J];
        else
            Jm = [Jm,zeros(6,1)];
        end
    end
end

