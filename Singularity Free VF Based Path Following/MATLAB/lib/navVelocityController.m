%Velocity Controller: That aligns the actual velocity vectors to desired
%velocity vectors: (Physical velocity can not be constant while using this. Only Total Velocity is
% VTot = sqrt(VPhy^2 + VVirt^2));
function [dv, omega] = navVelocityController(params, v, vd)
    unitV = v ./ norm(v); unitVd = vd ./ norm(vd);
    omega = params.k .* cross(unitV, unitVd);
    dv = cross(omega, unitV);

    % curvature constraint implementation
    N = v(1) * dv(2) - v(2) * dv(1);
    %without fixed minimum curvature
    % L = params.g * tan(params.phiMax) * sqrt(v(1)^2 + v(2)^2); 
    %with fixed maximum curvature
    KxyMax = params.g * tan(params.phiMax) / (0.75 * params.va)^2;
    L = KxyMax * (v(1)^2 + v(2)^2)^(3/2);

    if(abs(N) <= L)
        return;
    end

    alpha = L/abs(N);
    omega = alpha * omega;
    dv = cross(omega, unitV);
end