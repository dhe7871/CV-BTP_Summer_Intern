function handles = plotVF(params, region, varargin)
    p = inputParser;

    addRequired(p, "params", @(params) isstruct(params) && isfield(params, "fnS")...
        && isfield(params, "kappa") && isfield(params, "s"));
    addRequired(p, "region", @(region) isequal(size(region), [3, 2]) &&...
        region(1, 1) < region(1, 2) && region(2, 1) < region(2, 2) && region(3, 1) < region(3, 2));
    
    %first three, signifies a point from where the plane passes,
    %last three, signifies the plane perpendicular vector
    addOptional(p, "planes", [0, 0, 0, 0, 0, 1], @(planes) size(planes, 2) == 6 &&...
        all(any(planes(:, 4:6))));

    parse(p, params, region, varargin{:});
    planes = p.Results.planes;

    syms x y w;
    
    fnS = params.fnS; kappa = params.kappa; s = params.s;

    xCoordS = fnS.x; xCoordM = matlabFunction(xCoordS, "Vars", w);
    yCoordS = fnS.y; yCoordM = matlabFunction(yCoordS, "Vars", w);

    wCoord = region(3, 1):0.001:region(3, 2);
    xCoord = xCoordM(wCoord);
    yCoord = yCoordM(wCoord);

    figure(1);
    hold on; grid on; grid minor;
    view(45, 30); axis equal
    xlim(region(1, :)); ylim(region(2, :)); zlim(region(3, :));

    handles.plotVF.curveHigh = plot3(xCoord, yCoord, wCoord, "LineWidth", 1.5, "Color", "#0099ff");
    handles.plotVF.curvePhy = plot(xCoord, yCoord, "LineWidth", 1.5, "Color", "#ff5500");

    %Vector Field Plane

    
    %Gives a function that calculates vector field
    VFAtPoint = createVFAtPointMatFunc(fnS, s, kappa);
    for i = 1:size(planes, 1)
        if(planes(i, 6) == 0)
            [Y, W] = meshgrid(linspace(region(2, 1), region(2, 2), 25),...
                linspace(region(3, 1), region(3, 2), 25));

            %remaining axis coordinates
            if(planes(i, 5) == 0)
                X = planes(i, 4) .* ones(size(W));
            else
                X = Y;
                planeEq = planes(i, 4:6) * ([x, y, w]' - planes(i, 1:3)') == 0;
                planeYM = matlabFunction(solve(planeEq, y), "Vars", [X, w]);
                Y = planeYM(X, W) .* ones(size(W));
            end
        else
            planeEq = planes(i, 4:6) * ([x, y, w]' - planes(i, 1:3)') == 0;
            planeWM = matlabFunction(solve(planeEq, w), "Vars", [x, y]);

            [X, Y] = meshgrid(linspace(region(1, 1), region(1, 2), 25),...
                linspace(region(2, 1), region(2, 2), 25));
            W = planeWM(X, Y) .* ones(size(X));         % multiplication with ones is done
                                                        % to ensure the dimensions are same as X
        end

        
        handles.plotVF.VFPlane = surf(X, Y, W, "FaceAlpha", "0.2", "FaceColor", "#0099ff", "EdgeColor","none");
    
        %Vector Field Calculation
        VFOnPlane = VFAtPoint(X, Y, W);
    
        % disp(VFOnPlane);
        normVFOnPlane = VFOnPlane...
            ./ sqrt(VFOnPlane(:, :, 1).^2 + VFOnPlane(:, :, 2).^2 + VFOnPlane(:, :, 3).^2);
    
        % disp(normVFOnPlane);
        handleName = sprintf("planeQuiver%d", i);
        handles.plotVF.(handleName) = quiver3(X, Y, W, normVFOnPlane(:, :, 1), normVFOnPlane(:, :, 2),...
            normVFOnPlane(:, :, 3), "Color", "#ffe864");
    end

    xlabel("x[m]");
    ylabel("y[m]");
    zlabel("Parameter, w[m]");

    legend([handles.plotVF.curveHigh, handles.plotVF.curvePhy, handles.plotVF.planeQuiver1],...
        ["Higher Dimensional Curve", "Physical Curve", "Higher Dimensional VFs"]);
    hold off;
end