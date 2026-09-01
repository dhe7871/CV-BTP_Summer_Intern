function VF = createVFAtPointMatFunc(fnS, s, kappa)
    x = sym("x"); y = sym("y"); w = sym("w");

    f1w = fnS.x; f2w = fnS.y;
    dfdw = cell(2, 1);
    dfdw{1} = matlabFunction(diff(f1w, w), "Vars", w);
    dfdw{2} = matlabFunction(diff(f2w, w), "Vars", w);

    phiM = cell(2, 1);
    phi1S = x - f1w; phiM{1} = matlabFunction(phi1S, "Vars", [x, y, w]);
    phi2S = y - f2w; phiM{2} = matlabFunction(phi2S, "Vars", [x, y, w]);

    function VF = fieldGen(X, Y, W)
        sizeX = size(X); sizeY = size(Y); sizeW = size(W);
        ndim = numel(sizeX);

        if(~isequal(sizeX, sizeY) || ~isequal(sizeY, sizeW) || ndim > 3 )
            error("The size of the inputs X, Y, W must be same...");
        end
        
        phi1 = phiM{1}(X, Y, W);
        phi2 = phiM{2}(X, Y, W);
        
        fPrime = cell(2, 1);
        fPrime{1} = dfdw{1}(W);
        fPrime{2} = dfdw{2}(W);

        tempVF = cell(3, 1);
        for i = 1:3
            if(i == 3)
                tempVF{i} = s * sech(kappa(1) * phi1) .* sech(kappa(2) * phi2)...
                    + tanh(kappa(1) * phi1) .* fPrime{1} + tanh(kappa(2) * phi2) .* fPrime{2};
            else
                tempVF{i} =  s * sech(kappa(1) * phi1) .* sech(kappa(2) * phi2) .* fPrime{i}...
                    - tanh(kappa(i) * phiM{i}(X, Y, W));
            end
        end

        
        
        VF = zeros([sizeX, 3]);
        if(ndim == 2)
            VF(:, :, 1) = tempVF{1}; VF(:, :, 2) = tempVF{2}; VF(:, :, 3) = tempVF{3};
        elseif(ndim == 3)
            VF(:, :, :, 1) = tempVF{1}; VF(:, :, :, 2) = tempVF{2}; VF(:, :, :, 3) = tempVF{3};
        end
    end

    VF = @(X, Y, W) fieldGen(X, Y, W);
end