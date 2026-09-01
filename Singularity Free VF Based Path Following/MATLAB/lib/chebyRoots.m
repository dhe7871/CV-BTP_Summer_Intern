function cRoots = chebyRoots(f, interval, varargin)
    p = inputParser;

    addRequired(p, "f", @(f) isa(f, "sym") && isscalar(symvar(f)));
    addRequired(p, "interval", ...
        @(interval) (isequal(size(interval), [1, 2]) || isequal(size(interval), [2, 1]))...
        && all(isnumeric(interval), "all") && interval(1) < interval(2));
    
    addOptional(p, "tol", 1e-6, @(tol) isnumeric(tol) && tol > 0);
    addOptional(p, "minDegreeChebyPoly", 10, @(n) isnumeric(n) && n >= 10 && mod(n, 1) == 0)
    addOptional(p, "maxChebyApproxErr", 1e-10, @(mcae) isnumeric(mcae) && mcae > 0);

    parse(p, f, interval, varargin{:});
    tol = p.Results.tol;        %Tolerance for neglecting 'imaginary roots'
    n = p.Results.minDegreeChebyPoly;
    maxChebyApproxErr = p.Results.maxChebyApproxErr;



    syms t;
    x = symvar(f);
    
    try
        fPoly = sym2poly(f);
        isfPolynomial = true;
    catch
        isfPolynomial = false;
    end

    if(isfPolynomial)
        rts = roots(fPoly);
        cRoots = real(rts(abs(imag(rts)) < tol));
        return;
    end

    fx = f;
    fxm = matlabFunction(fx, "Vars", x);

    intervals = zeros(10, 2);
    intervals(1, :) = [interval(1), interval(2)];
    nIntervals = 1;
    degIntervalSplit = 40;

    cRoots = [];

    i = 1;
    while(i <= nIntervals)
        % fprintf("\nCalculating for Interval: [%f, %f]\n", intervals(i, 1), intervals(i, 2));

        a = intervals(i, 1); b = intervals(i, 2);
        ft = subs(fx, x, (b - a)*t/2 + (a + b)/2);
        ftm = matlabFunction(ft, "Vars", t);
        
        firstTimeFlag = true;
        iter = 1;
        while(n <= degIntervalSplit)
            k = 0:n;
            if(firstTimeFlag)
                tk = cos(k*pi./n);
                yk = ftm(tk);

                firstTimeFlag = false;
            else
                temp = tk;
                tk = zeros(1, n + 1);
                tk(1:2:end) = temp;
                tk(2:2:end) = cos(k(2:2:end)*pi./n);

                temp = yk;
                yk = zeros(1, n + 1);
                yk(1:2:end) = temp;
                yk(2:2:end) = ftm(tk(2:2:end));

                temp = [];
            end
            
            cj = c(0:n, yk, n);
            chebyApproxErr = sum(abs(cj((end - 4):end)), 2);
            if(chebyApproxErr > maxChebyApproxErr)
                % fprintf("Iteration: %d, Degree(n): %d, targetError: %e, currentError: %e\n",...
                %     iter, n, maxChebyApproxErr, chebyApproxErr);

                n = n * 2;
                iter = iter + 1;
                continue;
            end

            %mid-point(of 1st two nodes) sanity check for aliasing(false convergence)
            k = 0:(n - 1);
            tTest = cos((k + 1/2) .* pi./n);
            % tTest = ([tk, 0] + [0, tk])./2;
            yTrue = ftm(tTest);
            
            yApprox = 0.5 * cj(1);
            for idx = 1:(n- 1)
                yApprox = yApprox + cj(idx + 1) .* cos(idx .* acos(tTest));
            end
            yApprox = yApprox + 0.5 * cj(end) .* cos(n .* acos(tTest));
            
            midPointErrSum = sum(abs(yTrue - yApprox), 2);
            if(midPointErrSum > maxChebyApproxErr)
                % fprintf("Iteration: %d, Degree(n): %d, targetError: %e, currentError: %e, midPointErrorSum: %e\n" + ...
                % "|FALSE CONVERGENCE. MID POINT TEST FAILED...|\n",...
                % iter, n, maxChebyApproxErr, chebyApproxErr, midPointErrSum);

                n = n * 2;
                iter = iter + 1;
                continue;
            end

            % fprintf("Iteration: %d, Degree(n): %d, targetError: %e, currentError: %e, midPointErrorSum: %e\n",...
            %     iter, n, maxChebyApproxErr, chebyApproxErr, midPointErrSum);
            break;
        end
        
        if(n > degIntervalSplit)
            n = p.Results.minDegreeChebyPoly;
            intervals(i, :) = [a, (a + b)/2];

            if(nIntervals == size(intervals, 1))
                intervals = [intervals', zeros(2, 10)]';                
            end

            intervals(nIntervals + 1, :) = [(a + b)/2, b];
            nIntervals = nIntervals + 1;

            % warning("DIDN'T CONVERGE. SPLITTING INTERVAL...");

            % fprintf("\nIntervals: \n");
            % disp(intervals(1:nIntervals, :));
            continue;
        end


        % PRUNE NUMERICAL NOISE BEFORE MATRIX CREATION
        prune_tol = maxChebyApproxErr; 
        while abs(cj(end)) < prune_tol && length(cj) > 3
            cj(end) = [];
        end
        
        %Directly finding the roots without computing the Chebyshev series.
        %i.e. using the "Colleagues Matrix" approach
        clgMat = createColleaguesMatrix(cj);
        rtsComplex = eig(clgMat);
        
        rtsReal = real(rtsComplex(abs(imag(rtsComplex)) < tol));
        tRoots = rtsReal(rtsReal >= -1 & rtsReal <= 1);
        
        if(numel(tRoots) ~= 0)
            xRoots = sort([a, (b - a)*tRoots'/2 + (a + b)/2, b])';

            refinedRoots = zeros(length(xRoots), 1);
            lenRefinedRoots = 0; isZeroARoot = false;
            rIdx = 1;
            for r = xRoots'
                rInt = getMustHaveRootInterval(rIdx, xRoots, fxm);  %always supply the sorted(ascending) xRoots
                rr = fzero(fxm, rInt);

                if(all(refinedRoots ~= rr))
                    refinedRoots(lenRefinedRoots + 1) = rr;
                    lenRefinedRoots = lenRefinedRoots + 1;
                elseif(rr == 0 && ~isZeroARoot)
                    isZeroARoot = true;
                    lenRefinedRoots = lenRefinedRoots + 1;
                end

                rIdx = rIdx + 1;
            end
            refinedRoots = refinedRoots(1:lenRefinedRoots);
            
            cRoots = [cRoots', refinedRoots']';
            
            % fprintf("CONVERGED...\n");
            % fprintf("Roots: \n");
            % disp(refinedRoots);
           
        % else
        %     fprintf("CONVERGED. BUT no roots IN THIS INTERVAL...\n");
        end
        
        n = p.Results.minDegreeChebyPoly;
        i = i + 1;
    end

    cRoots = sort(cRoots(cRoots >= interval(1) & cRoots <= interval(2)));
    return;
end







function coeff = c(j, yk, n)
    if(~all(mod(j, 1) == 0, "all") || ~all(j >= 0 & j <= n, "all"))
        error("The input to calculate the coefficients (Cj), must be an Integer and\n" + ...
            "and must be within 0 and %d.\n", n);
    end
    if(numel(size(yk)) == 2 && any(size(yk) == 1))
        yk = reshape(yk, [1, numel(yk)]); %make sure yk is a 'row' vector;
    else
        error("The 2nd input (yk) must be a vector...")
    end

    k = 0:n;

    if(isscalar(j))
        coeff = (2/n) * yk * [0.5, cos(j * k(2:(end - 1)) * pi/n), 0.5*(-1)^j]';
    elseif(numel(size(j)) == 2 && any(size(j) == 1))
        isInputColVector = (find(size(j) == 1) == 2);
        
        j = reshape(j, [numel(j), 1]);  %make sure the j vector supplied to next step is a col vector
        cosines = cos(j .* k(2:(end - 1)) * pi/n);
        wt = [0.5 * ones(size(cosines, 1), 1), cosines, 0.5*(-1).^j]';

        coeff = (2/n) * yk * wt;        %row vector

        if(isInputColVector)
            coeff = coeff';             %if j is supplied as a column vector, 
                                        % then send the output also as column vector
        end
    else
        error("The input can only be a scalar or a vector...\n");
    end
end

function C = createColleaguesMatrix(cj)
    %Pn(x) = a0*T0(x) + a1*T1(x) + ....... + an*Tn(x);
    a = cj; a(1) = a(1)/2; a(end) = a(end)/2;

    n = numel(cj) - 1;
    C = zeros(n, n);

    %first row
    C(1, 2) = 1;
    
    %row 2 to (n - 1)
    for i = 2:(n - 1)
        C(i, (i - 1):(i + 1)) = [0.5, 0, 0.5];
    end

    %last row
    for j = 1:n
        C(n, j) = -a(j)/(2 * a(end));
        
        if(j == n - 1)
            C(n, j) = C(n, j) + 0.5;
        end
    end
end

function rInt = getMustHaveRootInterval(rIdx, xRoots, fxm)
    r = xRoots(rIdx);
    
    if(rIdx == 1)
        rightGap = xRoots(rIdx + 1) - r;
        leftGap = rightGap;
    elseif(rIdx == length(xRoots))
        leftGap = r - xRoots(rIdx - 1);
        rightGap = leftGap;
    else
        leftGap = r - xRoots(rIdx - 1);
        rightGap = xRoots(rIdx + 1) - r;
    end

    percent = 1;
    while(percent <= 64)
        left = r - leftGap * percent / 100;
        right = r + rightGap *percent / 100;

        if(fxm(left) * fxm(right) < 0)
            rInt = [left, right];
            return;
        end

        percent = percent * 2;
    end
    
    rInt = r;
    return;
end