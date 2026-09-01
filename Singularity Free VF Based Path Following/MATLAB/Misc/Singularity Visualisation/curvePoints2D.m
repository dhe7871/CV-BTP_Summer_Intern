function curve = curvePoints2D(F, x0, steps, varargin)
    p = inputParser;
    addRequired(p, "F", @(F) isa(F, "function_handle"));
    addRequired(p, "x0", @(x0) size(x0, 1) == 2 && size(x0, 2) == 1);
    addRequired(p, "steps", @(s) isscalar(s) && s > 0)
    addParameter(p, "Direction", "both", @(dir) dir == "forward" || dir == "backward" || dir == "both");
    addParameter(p, "Truncate", true, @islogical)
    
    parse(p, F, x0, steps, varargin{:});
    dir = p.Results.Direction;
    truncate = p.Results.Truncate;
    
    
    fprintf("Started Calculating Curve Points....\n");
    fun = @(x) F(x(1), x(2));

    opts = optimoptions("fsolve", "Display", "none", "Algorithm","levenberg-marquardt");
    p0 = fsolve(fun, x0, opts);

    h = 0.05;
    tolClose = 0.1; %tolerance must be greater than step size 'h' to avoid skipping

    if(dir == "forward" || dir == "backward")
        progressbar("Initialize", [1, steps]);
        curve = zeros(2, steps);
        p = p0;
        
        planeNormDir = [0, 0, 1]' .* (dir == "forward") + [0, 0, -1]' .* (dir == "backward");
        for k = 1:steps
            gradF = grad(fun, p);
            
            tdir = cross([gradF', 0]', planeNormDir);
            ntdir = tdir(1:2) ./ norm(tdir(1:2));

            x = p + ntdir .* h;
            p = fsolve(fun, x, opts);

            curve(:, k) = p;
            if(truncate && norm(p - p0) < tolClose && k > steps/10)
                curve = [p0, curve(:, k)];
                
                progressbar(steps);
                fprintf("Curve Points Calculation Complete...\n\n");
                return;
            end
            progressbar(k);
        end
        curve = [p0, curve];
    else
        progressbar("Initialize", [1, floor(steps/2)]);

        curveFwd = zeros(2, floor(steps/2));
        curveBwd = zeros(2, floor(steps/2));
    
        pF = p0;
        pB = p0;
        for k = 1:size(curveFwd, 2)
            gradFFwd = grad(fun, pF);
            gradFBwd = grad(fun, pB);
    
            tdirFwd = cross([gradFFwd', 0]', [0, 0, 1]');
            ntdirFwd = tdirFwd(1:2) ./ norm(tdirFwd(1:2));
    
            tdirBwd = cross([gradFBwd', 0]', [0, 0, -1]');
            ntdirBwd = tdirBwd(1:2) ./ norm(tdirBwd(1:2));
    
            xFwd = pF + ntdirFwd .* h;
            xBwd = pB + ntdirBwd .* h;
    
            pF = fsolve(fun, xFwd, opts);
            pB = fsolve(fun, xBwd, opts);
            
            curveFwd(:, k) = pF; curveBwd(:, k) = pB;
            if(truncate && norm(pF - pB) < tolClose && k > steps/10)
                curve = [flip(curveFwd(:, 1:k)')', p0, curveBwd(:, 1:k)];
                
                progressbar(floor(steps/2));
                fprintf("Curve Points Calculation Complete...\n\n");
                return;
            end
            progressbar(k);
        end
        curve = [flip(curveFwd')', p0, curveBwd];
    end
    fprintf("Curve Points Calculation Complete...\n\n");
end

%Gradient via finite Difference
function g = grad(fun, p)
    h = 1e-6;
    
    pxi = [p(1) + h; p(2)]; pxd = [p(1) - h; p(2)];
    pyi = [p(1); p(2) + h]; pyd = [p(1); p(2) - h];
    g = [
        (fun(pxi) - fun(pxd))/(2*h);
        (fun(pyi) - fun(pyd))/(2*h);
    ];
end