function A=dh_matrix(in_vector)    
    % input: dh parameter, in_vector=[theta,d,alpha,a]
    % theta,alpha: rad
    % d,a: meter
    assert(isequal(size(in_vector),[1,4]) || isequal(size(in_vector),[4,1]), ...
           "Please check the input, should be a 4 dimensional vector in column or row")
    theta = in_vector(1);
    alpha = in_vector(3);
%    theta=theta*pi/180;
%    alpha=alpha*pi/180;
    
    d = in_vector(2);
    a = in_vector(4);
    
    A =[cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
        sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                 0,             sin(alpha),             cos(alpha),            d;
                 0,                      0,                      0,             1];
    
end