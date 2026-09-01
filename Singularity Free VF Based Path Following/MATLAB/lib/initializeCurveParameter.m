function w0 = initializeCurveParameter(p0, fnS, interval)
    x0 = p0(1); y0 = p0(2);

    w = symvar(fnS.x);
    
    f1 = fnS.x; f2 = fnS.y;
    df1 = diff(f1, 1, w); df2 = diff(f2, 1, w);
    ddf1 = diff(df1, 1, w); ddf2 = diff(df2, 1, w);

    F = df1 * (f1 - x0) + df2 * (f2 - y0);

    G = 2 * ((f1 - x0) * ddf1 + (f2 - y0) * ddf2) + 2 * (df1^2 + df2^2);
    Gm = matlabFunction(G, "Vars", w);

    FRoots = chebyRoots(F, interval);
    
    %minima points only
    FRoots = FRoots(Gm(FRoots) > 0);

    d2 = (f1 - x0)^2 + (f2 - y0)^2;
    d2m = matlabFunction(d2, "Vars", w);
    
    distances = d2m(FRoots);
    
    tol = 1e-9;
    FRoots = FRoots(abs(distances - min(distances)) < tol);
    [~, idx]= min(abs(FRoots));
    
    w0 = FRoots(idx);
end