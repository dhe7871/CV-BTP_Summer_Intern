%Finding the shortest distance to a curve (i.e. length of the normal to the curve)
function [xs, ys] = normLen(Fn, dFdx, dFdy, p)
    a = p(1); b = p(2);
    
    fun = @(v)[
        Fn(v(1), v(2));
        (v(1) - a)*dFdy(v(1), v(2)) - (v(2) - b)*dFdx(v(1), v(2))
        ];
    
    opts = optimoptions('fsolve','Display','off');
    
    sol = fsolve(fun,[a b],opts);
    
    xs = sol(1);
    ys = sol(2);
end