function [handles, updatePlotTrajectory]= initializePlotTrajectory(params, handles)
    p0 = params.p0;
    region = params.region; fnM = params.fnM; 

    figure;
    subplot(1, 2, 1);
    hold on; grid on; grid minor; axis equal;
    
    W = region(3, 1):0.01:region(3, 2);
    X = fnM.x(W); Y = fnM.y(W);
    handles.plotPF.curveDesired = plot(X, Y,"LineStyle", "--", "LineWidth", 1.5, "Color", "#ff5500");

    handles.plotPF.initialPos = scatter(p0(1), p0(2), "Marker", "o", "SizeData", 80);
    
    handles.plotPF.currentVTP = scatter(fnM.x(p0(3)), fnM.y(p0(3)), "Marker", "x", "SizeData", 80,...
        "MarkerEdgeColor", "#ffe864", "LineWidth", 1.5);
    handles.plotPF.curveFollow = plot(p0(1), p0(2), "LineWidth", 1.5, "Color", "#0099ff");
    handles.plotPF.currentPos = scatter(p0(1), p0(2), "Marker", "^", "SizeData", 80,...
        "MarkerEdgeColor", "#0099ff", "LineWidth", 1.5);
    
    xlabel("x[m]");
    ylabel("y[m]");
    title("Projected Path Following");

    legend([ handles.plotPF.curveDesired, ...
        handles.plotPF.curveFollow, ...
        handles.plotPF.initialPos, ...
        handles.plotPF.currentPos, ...
        handles.plotPF.currentVTP...
        ], ["Desired Curve", "Obstacle To Avoid", "Actual Followed curve",...
        "Initial Position", "Current Position", "VTP"]);
    
    hold off;
    
    subplot(1, 2, 2);
    hold on; grid on; grid minor; axis equal;
    view(45, 30);
    handles.plotPF.curveDesiredHigh = plot3(X, Y, W,"LineStyle", ":", "LineWidth", 1.5, "Color", "#ff5500");
    handles.plotPF.initialPosHigh = scatter3(p0(1), p0(2), p0(3), "Marker", "o", "SizeData", 80);
    
    handles.plotPF.currentVTPHigh = scatter3(fnM.x(p0(3)), fnM.y(p0(3)), p0(3), "Marker", "x", "SizeData", 80,...
        "MarkerEdgeColor", "#ffe864", "LineWidth", 1.5);
    handles.plotPF.curveFollowHigh = plot3(p0(1), p0(2), p0(3), "LineWidth", 1.5, "Color", "#0099ff");
    handles.plotPF.currentPosHigh = scatter3(p0(1), p0(2), p0(3), "Marker", "^", "SizeData", 80,...
        "MarkerEdgeColor", "#0099ff", "LineWidth", 1.5);
    
    xlabel("x[m]");
    ylabel("y[m]");
    zlabel("Parameter, w");
    title("Path Following in Extended dimensions");

    legend([ handles.plotPF.curveDesiredHigh, ...
        handles.plotPF.curveFollowHigh, ...
        handles.plotPF.initialPosHigh, ...
        handles.plotPF.currentPosHigh, ...
        handles.plotPF.currentVTPHigh...
        ], ["Desired Curve", "Actual Follow curve", "Initial Position", "Current Position", "VTP"]);
    hold off;

    function update(states, i)
        set(handles.plotPF.curveFollow, "XData", states.state(1, 1:(i + 1)));
        set(handles.plotPF.curveFollow, "YData", states.state(2, 1:(i + 1)));

        set(handles.plotPF.currentPos, "XData", states.state(1, i + 1));
        set(handles.plotPF.currentPos, "YData", states.state(2, i + 1));

        set(handles.plotPF.currentVTP, "XData", fnM.x(states.state(3, i)));
        set(handles.plotPF.currentVTP, "YData", fnM.y(states.state(3, i)));

        set(handles.plotPF.curveFollowHigh, "XData", states.state(1, 1:(i + 1)));
        set(handles.plotPF.curveFollowHigh, "YData", states.state(2, 1:(i + 1)));
        set(handles.plotPF.curveFollowHigh, "ZData", states.state(3, 1:(i + 1)));

        set(handles.plotPF.currentPosHigh, "XData", states.state(1, i + 1));
        set(handles.plotPF.currentPosHigh, "YData", states.state(2, i + 1));
        set(handles.plotPF.currentPosHigh, "ZData", states.state(3, i + 1));

        set(handles.plotPF.currentVTPHigh, "XData", fnM.x(states.state(3, i)));
        set(handles.plotPF.currentVTPHigh, "YData", fnM.y(states.state(3, i)));
        set(handles.plotPF.currentVTPHigh, "ZData", states.state(3, i));

        drawnow limitrate;
    end

    updatePlotTrajectory = @(states, i) update(states, i);
end