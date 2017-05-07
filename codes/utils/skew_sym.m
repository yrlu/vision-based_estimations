function [r_x] = skew_sym(r)
r_x = [0, -r(3), r(2);
       r(3), 0, -r(1);
       -r(2), r(1), 0];
end