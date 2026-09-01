function handles = plotParams(params, handles, states)
    % states.state: [x, y, w, dx, dy, dw, ddx, ddy, ddw, Vg, Kxy, r]'
    % states.miscState: [omega1, omega2, omega3, chi, dChi]';

    t = 0:params.dt:params.simTime;

    controller = params.controller;
    if(controller == "Vel")
        figure;
        subplot(2, 2, 1);
        hold on; grid on; grid minor;
        VPhy = sqrt(states.state(4, :).^2 + states.state(5, :).^2)';
        VVirt = states.state(6, :)';
        VTot = sqrt(VPhy.^2 + VVirt.^2);
        handles.plotParams.plotTotVel = plot(t, VTot, "LineStyle", "--", "LineWidth", 1.5);
        handles.plotParams.plotPhyVel = plot(t, VPhy, "LineWidth", 1.5);
        handles.plotParams.plotVirtVel = plot(t, VVirt, "LineWidth", 1.5);
        
        xlabel("Time[s]");
        ylabel("Velocities[m/s]");
        title("Variation of Different Velocities");
        legend([handles.plotParams.plotTotVel,...
            handles.plotParams.plotPhyVel,...
            handles.plotParams.plotVirtVel],...
            [sprintf("Total Velocity (V_a = %0.2f)", va),...
            "Physical Velocity", "Virtual velocity (\dot(w))"])
        hold off;
        
        subplot(2, 2, 2);
        hold on; grid on; grid minor
        handles.plotParams.plotOmegaX = plot(t, states.miscState(1, :), "LineWidth", 1.5);
        handles.plotParams.plotOmegaY = plot(t, states.miscState(2, :), "LineWidth", 1.5);
        handles.plotParams.plotOmegaZ = plot(t, states.miscState(3, :), "LineWidth", 1.5);
        
        xlabel("Time[s]");
        ylabel("\omega[rad/s]");
        title("Angular Velocity Variation");
        legend([handles.plotParams.plotOmegaX,...
            handles.plotParams.plotOmegaY,...
            handles.plotParams.plotOmegaZ],...
            ["\omega_x", "\omega_y", "\omega_z"]);
        hold off;
        
        subplot(2, 2, 3);
        hold on; grid on; grid minor;
        handles.plotParams.plotCurvatureXY = plot(t, states.state(11, :), "LineWidth", 1.5);
        
        % Kxylim = g*tan(phiMax)./(states.state(4, :).^2 + states.state(5, :).^2);
        Kxylim = (params.g * tan(params.phiMax) / (0.75 * params.va)^2) .* ones(size(states.state(4, :)));
        handles.plotParams.plotCurvatureXYMax = plot(t,  Kxylim,...
            "LineStyle", "--", "Color", "#ff0000", "LineWidth", 1);
        handles.plotParams.plotCurvatureXYMin = plot(t, -Kxylim,...
            "LineStyle", "--", "Color", "#ff0000", "LineWidth", 1);
        
        xlabel("Time[s]")
        ylabel("Curvature in XY Plane, \kappa_x_y");
        title("Curvature Variation");
        legend([handles.plotParams.plotCurvatureXY,...
            handles.plotParams.plotCurvatureXYMax],...
            ["Curvature (\kappa_x_y)", "Curvature Limits: (\kappa_x_y)_m_i_n and (\kappa_x_y)_m_a_x"])
        hold off;
        
        subplot(2, 2, 4);
        hold on; grid on; grid minor;
        handles.plotParams.plotDistFromVTP = plot(t, states.state(12, :), "LineWidth", 1.5);
        
        xlabel("Time[s]")
        ylabel("Distance From VTP, r[m]");
        title("Variation of distance between VTP and current position");
        hold off;
    elseif(controller == "Dist")
        figure;
        subplot(3, 2, 1);
        hold on; grid on; grid minor;
        handles.plotParams.plotGroundVelocity = plot(t, states.state(10, :), "LineWidth", 1.5);
        
        xlabel("Time[s]");
        ylabel("Ground Velocity, V_g[m/s]");
        title("Ground Velocity Variation");
        hold off;

        subplot(3, 2, 2);
        hold on; grid on; grid minor;
        handles.plotParams.plotCurvatureXY = plot(t, states.state(11, :), "LineWidth", 1.5);

        dChilim = params.g * tan(params.phiMax) / params.va;    %The constraint is given as the "Constant Course rate constraint"
        Kxylim = dChilim ./ states.state(10, :);
        handles.plotParams.plotCurvatureXYMax = plot(t,  Kxylim,...
            "LineStyle", "--", "Color", "#ff0000", "LineWidth", 1);
        handles.plotParams.plotCurvatureXYMin = plot(t,  -Kxylim,...
            "LineStyle", "--", "Color", "#ff0000", "LineWidth", 1);

        xlabel("Time[s]")
        ylabel("Curvature in XY Plane, \kappa_x_y");
        title("Curvature Variation");
        legend([handles.plotParams.plotCurvatureXY,...
            handles.plotParams.plotCurvatureXYMax],...
            ["Curvature (\kappa_x_y)", "Curvature Limits: (\kappa_x_y)_m_i_n and (\kappa_x_y)_m_a_x"])
        hold off;

        subplot(3, 2, 3);
        hold on; grid on; grid minor;
        handles.plotParams.plotCourseAngle = plot(t, states.miscState(4, :) * 180/pi, "LineWidth", 1.5);

        xlabel("Time[s]");
        ylabel("Course angle \chi[degrees]");
        title("Course angle variation");
        hold off;

        subplot(3, 2, 4);
        hold on; grid on; grid minor;
        handles.plotParams.plotCourseAngleRate = plot(t, states.miscState(5, :), "LineWidth", 1.5);

        dChilim = params.g * tan(params.phiMax) / params.va;
        handles.plotParams.plotdChiMax = plot([0, params.simTime],  [dChilim, dChilim],...
            "LineStyle", "--", "Color", "#ff0000", "LineWidth", 1);
        handles.plotParams.plotdChiMin = plot([0, params.simTime],  [-dChilim, -dChilim],...
            "LineStyle", "--", "Color", "#ff0000", "LineWidth", 1);

        xlabel("Time[s]");
        ylabel("Course angle rate $\dot{\chi}$[rad/s]", "Interpreter", "latex");
        title("Course angle rate variation");

        legend([handles.plotParams.plotCourseAngleRate, ...
            handles.plotParams.plotdChiMax], ["Course rate", "Course rate limit"]);
        hold off;

        subplot(3, 2, 5);
        hold on; grid on; grid minor;
        handles.plotParams.plotDistFromVTP = plot(t, states.state(12, :), "LineWidth", 1.5);

        xlabel("Time[s]")
        ylabel("Distance From VTP, r[m]");
        title("Variation of distance between VTP and current position");
        hold off;
    else
        error("Please select a Valid Controller(params.controller): 'Vel' or 'Dist'\n");
    end
end