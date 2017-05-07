function [A] = get_A(x,y,Zc)
    % generate the A matrix for estimating the v and omega
    % @input        x,y     nx1 coordinates in the image frame       
    %               Zc      Z coordinate w.r.t. camera frame
    % @output       A       2nx6 A matrix
    n = numel(x);
    f1 = @(x, y, Z) [-1./Z, zeros(n, 1), x./Z, x.*y, -(1+x.^2), y];
    f2 = @(x, y, Z) [zeros(n, 1), -1./Z, y./Z, (1+y.^2), -x.*y, -x];
    A = [f1(x,y,Zc); f2(x,y,Zc)];
end
