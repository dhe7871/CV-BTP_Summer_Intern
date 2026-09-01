function [vNew, chiNew, dChi] = navDistribController(params, v, vd, tol, kchi)
    %tol: Singularity Tolerance
    wind = params.wind; va = params.va; chi = atan2(v(2), v(1));
        
    Vdxy = sqrt(vd(1)^2 + vd(2)^2);
    if(Vdxy < tol)
        dChi = 0;
        return;
    end
    vdCap = vd ./ Vdxy;

    chid = atan2(vdCap(2), vdCap(1));
    dChi = kchi * wrapToPi(chid - chi);

    dChiMax = params.g * tan(params.phiMax) / va;
    dChi = max(min(dChi, dChiMax), -dChiMax);

    chiNew = wrapToPi(chi + dChi * params.dt);
    VgNew = wind(1) * cos(chiNew) + wind(2) * sin(chiNew) + sqrt(va^2 ...
        - (wind(1) * sin(chiNew) - wind(2) * cos(chiNew))^2);    
    vNew = [
        cos(chiNew);
        sin(chiNew);
        vdCap(3);
    ] .* VgNew;
end