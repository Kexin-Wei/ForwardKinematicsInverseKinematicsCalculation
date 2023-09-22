function r=QFromEuler(theta,n)
    % build the quaternion with (cos(theta/2),sin(theta/2)*n)
    % where n is a vector
    if isequal(size(n),[3,1])
        n=n'; % 1x3
    end
    n=n/norm(n); %normalize the rotate axis vector
    q=[cos(theta/2), sin(theta/2)*n];
    r=Quaternion(q);
end